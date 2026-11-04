#ifndef UR_DRIVER_ROS_COMPLIANCE_SCHEDULER_H_INCLUDED
#define UR_DRIVER_ROS_COMPLIANCE_SCHEDULER_H_INCLUDED

#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include <ros/time.h>
#include <ur_extra_msgs/ComplianceSchedule.h>
#include <ur_extra_msgs/ComplianceSummary.h>

#include "ur_robot_driver/comm/compliance_parameters.h"

namespace ur_driver
{
/*!
 * \brief Values of the compliant robot script's state register.
 */
enum class ComplianceScriptState : int32_t
{
  SERVOJ = 0,
  COMPLIANT = 1,
  REFUSED = 2,  ///< The script refused the profile values it received.
};

/*!
 * \brief What the compliant robot script reports each cycle, read back over
 * RTDE output registers, plus the TCP force.
 */
struct ComplianceScriptStatus
{
  // A ComplianceScriptState value; any other value counts as not compliant.
  int32_t state = 0;
  // A ComplianceSummary END_* value, written when compliance ends.
  int32_t end_reason = 0;
  // Planned minus actual TCP position in the tool frame [m].
  std::array<double, 3> deflection = { { 0.0, 0.0, 0.0 } };
  // TCP force magnitude [N].
  double force = 0.0;
};

/*!
 * \brief Holds the compliance schedule: says at each control cycle whether a
 * window is active and with which profile values, and builds one
 * ComplianceSummary per window from the robot script's status.
 *
 * Not thread-safe; the caller serializes access.
 */
class ComplianceScheduler
{
public:
  // The script stays compliant after a window ends only while it unloads the
  // spring (0.1 s); a summary still compliant this long after its window's
  // end closes with END_NOT_REPORTED.
  static constexpr double k_max_end_report_delay = 0.5;  // [s]

  /*!
   * \brief Replace the schedule.
   *
   * Summaries of the previous schedule's windows that have not ended are
   * closed as cancelled.
   *
   * \throws std::invalid_argument If `schedule` is invalid (window order,
   * timing, profile index or values); nothing changes then.
   */
  void setSchedule(const ur_extra_msgs::ComplianceSchedule& schedule, const ros::Time& time);

  /*!
   * \brief Clear the schedule, as an empty schedule would.
   */
  void clearSchedule(const ros::Time& time);

  /*!
   * \brief Profile values of the window active at `time`, or null outside
   * every window. Valid until the schedule next changes.
   */
  const comm::ComplianceParameters* findActiveParameters(const ros::Time& time) const;

  /*!
   * \brief Account the script's status at `time` into the open summaries.
   */
  void updateSummaries(const ros::Time& time, const ComplianceScriptStatus& status);

  /*!
   * \brief Remove and return the summaries closed so far, oldest first.
   */
  std::vector<ur_extra_msgs::ComplianceSummary> popClosedSummaries();

private:
  struct Window
  {
    ros::Time begin;
    ros::Time end;
    uint8_t profile;
  };

  void cancelOpenSummaries(const ros::Time& time);
  // Stamp the front open summary and move it to the closed ones.
  void closeFrontSummary(const ros::Time& time);

  std::vector<comm::ComplianceParameters> profiles_;
  std::vector<Window> windows_;
  // Ordered by window begin; the front one receives the script's status.
  std::deque<ur_extra_msgs::ComplianceSummary> open_summaries_;
  std::vector<ur_extra_msgs::ComplianceSummary> closed_summaries_;
};

/*!
 * \brief How loudly a closed summary should be logged.
 */
enum class ComplianceSummarySeverity
{
  NONE,     ///< Normal outcome: the window or a lift release ended compliance, or it was cancelled before it began.
  WARNING,  ///< Compliance ended early or never ran.
  ERROR,    ///< The robot script refused the profile.
};

ComplianceSummarySeverity classifyComplianceSummary(const ur_extra_msgs::ComplianceSummary& summary);

/*!
 * \brief One-line description of a summary's window and outcome, for logs.
 */
std::string describeComplianceSummary(const ur_extra_msgs::ComplianceSummary& summary);

}  // namespace ur_driver

#endif  // UR_DRIVER_ROS_COMPLIANCE_SCHEDULER_H_INCLUDED
