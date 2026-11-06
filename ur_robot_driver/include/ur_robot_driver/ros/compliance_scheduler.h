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
};

/*!
 * \brief What the compliant robot script reports each cycle, read back over
 * RTDE output registers, plus the TCP force.
 */
struct ComplianceScriptStatus
{
  // A ComplianceScriptState value; any other value counts as not compliant.
  int32_t state = 0;
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
  // The script leaves compliance within a few cycles of the window's end
  // (one RTDE round trip); a summary still compliant this long after its
  // window's end closes with END_NOT_REPORTED.
  static constexpr double k_max_end_report_delay = 0.5;  // [s]

  /*!
   * \brief Apply `schedule` from its stamp, as a joint trajectory controller
   * applies a new goal from its start time; a zero stamp means `time`.
   *
   * Held windows that end by then are kept, one running across it is cut
   * there, later ones are dropped; then the new windows are added. A window
   * cut or dropped at or before `time` has its summary closed as cancelled.
   *
   * \throws std::invalid_argument If `schedule` is invalid (window order,
   * timing, profile index or values, or windows without a stamp); nothing
   * changes then.
   */
  void setSchedule(const ur_extra_msgs::ComplianceSchedule& schedule, const ros::Time& time);

  /*!
   * \brief Clear every window from `time`, as an empty schedule stamped 0
   * received at `time` would.
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
    uint8_t profile;  // Index into the schedule it came from, for its summary.
    comm::ComplianceParameters parameters;
  };

  // Keep what ends by `cutoff`, cut what runs across it, drop the rest.
  void cutWindows(const ros::Time& cutoff, const ros::Time& time);
  void closeCancelledSummary(ur_extra_msgs::ComplianceSummary& summary, const ros::Time& time);
  // Stamp the front open summary and move it to the closed ones.
  void closeFrontSummary(const ros::Time& time);

  // Ordered and non-overlapping.
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
  NONE,     ///< Normal outcome: compliance ended with the window, or the window was cancelled before it began.
  WARNING,  ///< Compliance ended early or never ran.
};

ComplianceSummarySeverity classifyComplianceSummary(const ur_extra_msgs::ComplianceSummary& summary);

/*!
 * \brief One-line description of a summary's window and outcome, for logs.
 */
std::string describeComplianceSummary(const ur_extra_msgs::ComplianceSummary& summary);

}  // namespace ur_driver

#endif  // UR_DRIVER_ROS_COMPLIANCE_SCHEDULER_H_INCLUDED
