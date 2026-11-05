#include "ur_robot_driver/ros/compliance_scheduler.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace ur_driver
{
using ur_extra_msgs::ComplianceSchedule;
using ur_extra_msgs::ComplianceSummary;

namespace
{
void requireRepresentable(double value, const char* name, size_t profile_idx)
{
  if (!comm::isComplianceValueRepresentable(value))
  {
    std::ostringstream message;
    message << "profile " << profile_idx << " " << name << " " << value << " is not finite or out of range";
    throw std::invalid_argument(message.str());
  }
}

comm::ComplianceParameters parametersFromProfile(const ur_extra_msgs::ComplianceProfile& profile, size_t profile_idx)
{
  comm::ComplianceParameters parameters;
  for (size_t i = 0; i < parameters.stiffness.size(); ++i)
  {
    requireRepresentable(profile.stiffness[i], "stiffness", profile_idx);
    requireRepresentable(profile.damping[i], "damping", profile_idx);
    parameters.stiffness[i] = profile.stiffness[i];
    parameters.damping[i] = profile.damping[i];
  }
  requireRepresentable(profile.max_force, "max_force", profile_idx);
  parameters.max_force = profile.max_force;
  return parameters;
}

const char* describeEndReason(uint8_t end_reason)
{
  switch (end_reason)
  {
    case ComplianceSummary::END_WINDOW:
      return "window ended";
    case ComplianceSummary::END_CANCELLED:
      return "cancelled";
    case ComplianceSummary::END_EARLY:
      return "robot script stopped being compliant before the window ended";
    case ComplianceSummary::END_NOT_REPORTED:
      return "robot script did not report the end";
    default:
      return "unknown end reason";
  }
}

const char* describeNotCompliantReason(uint8_t not_compliant_reason)
{
  switch (not_compliant_reason)
  {
    case ComplianceSummary::NOT_COMPLIANT_CANCELLED:
      return "cancelled before it began";
    case ComplianceSummary::NOT_COMPLIANT_NOT_REPORTED:
      return "robot script did not report compliant";
    default:
      return "unknown reason";
  }
}

}  // namespace

void ComplianceScheduler::setSchedule(const ComplianceSchedule& schedule, const ros::Time& time)
{
  // Validate everything before changing anything.
  std::vector<comm::ComplianceParameters> profiles;
  for (size_t i = 0; i < schedule.profiles.size(); ++i)
  {
    profiles.push_back(parametersFromProfile(schedule.profiles[i], i));
  }
  if (!schedule.windows.empty() && schedule.stamp.isZero())
  {
    throw std::invalid_argument("windows without a stamp");
  }
  std::vector<Window> windows;
  for (size_t i = 0; i < schedule.windows.size(); ++i)
  {
    const ur_extra_msgs::ComplianceWindow& window = schedule.windows[i];
    std::ostringstream where;
    where << "window " << i << " [" << window.begin << ", " << window.end << "] s";
    if (window.begin < ros::Duration(0.0) || window.end <= window.begin)
    {
      throw std::invalid_argument(where.str() + ": begin must be >= 0 and end > begin");
    }
    if (window.profile >= profiles.size())
    {
      throw std::invalid_argument(where.str() + ": profile " + std::to_string(window.profile) + " of " +
                                  std::to_string(profiles.size()));
    }
    if (i > 0 && window.begin < schedule.windows[i - 1].end)
    {
      throw std::invalid_argument(where.str() + ": overlaps or precedes the previous window");
    }
    windows.push_back(Window{ schedule.stamp + window.begin, schedule.stamp + window.end, window.profile });
  }

  cancelOpenSummaries(time);
  profiles_ = profiles;
  windows_ = windows;
  for (const Window& window : windows_)
  {
    ComplianceSummary summary;
    summary.window_begin = window.begin;
    summary.window_end = window.end;
    summary.profile = window.profile;
    open_summaries_.push_back(summary);
  }
}

void ComplianceScheduler::clearSchedule(const ros::Time& time)
{
  cancelOpenSummaries(time);
  profiles_.clear();
  windows_.clear();
}

const comm::ComplianceParameters* ComplianceScheduler::findActiveParameters(const ros::Time& time) const
{
  for (const Window& window : windows_)
  {
    if (window.begin <= time && time < window.end)
    {
      return &profiles_[window.profile];
    }
  }
  return nullptr;
}

void ComplianceScheduler::updateSummaries(const ros::Time& time, const ComplianceScriptStatus& status)
{
  // Once the next window begins, the script's status belongs to it.
  while (open_summaries_.size() >= 2 && time >= open_summaries_[1].window_begin)
  {
    ComplianceSummary& superseded = open_summaries_.front();
    if (superseded.was_compliant)
    {
      superseded.end_reason = ComplianceSummary::END_WINDOW;
    }
    else
    {
      superseded.not_compliant_reason = ComplianceSummary::NOT_COMPLIANT_NOT_REPORTED;
    }
    closeFrontSummary(time);
  }
  if (open_summaries_.empty() || time < open_summaries_.front().window_begin)
  {
    return;
  }
  ComplianceSummary& summary = open_summaries_.front();
  summary.max_force = std::max(summary.max_force, status.force);

  if (status.state == static_cast<int32_t>(ComplianceScriptState::COMPLIANT))
  {
    if (!summary.was_compliant)
    {
      summary.was_compliant = true;
      summary.compliance_begin = time;
    }
    summary.compliance_end = time;
    summary.max_deflection_z = std::max(summary.max_deflection_z, status.deflection[2]);
    summary.max_deflection_xy =
        std::max(summary.max_deflection_xy, std::hypot(status.deflection[0], status.deflection[1]));
    summary.exit_deflection_z = status.deflection[2];
    if (time >= summary.window_end + ros::Duration(k_max_end_report_delay))
    {
      summary.end_reason = ComplianceSummary::END_NOT_REPORTED;
      closeFrontSummary(time);
    }
    return;
  }
  if (summary.was_compliant)
  {
    // The script ends compliance only when the window ends.
    summary.end_reason =
        time >= summary.window_end ? ComplianceSummary::END_WINDOW : ComplianceSummary::END_EARLY;
    closeFrontSummary(time);
    return;
  }
  if (time >= summary.window_end)
  {
    summary.not_compliant_reason = ComplianceSummary::NOT_COMPLIANT_NOT_REPORTED;
    closeFrontSummary(time);
  }
}

std::vector<ComplianceSummary> ComplianceScheduler::popClosedSummaries()
{
  std::vector<ComplianceSummary> closed;
  closed.swap(closed_summaries_);
  return closed;
}

void ComplianceScheduler::cancelOpenSummaries(const ros::Time& time)
{
  // Windows that have ended keep their summary open until the script reports
  // how compliance ended (e.g. after the unload).
  std::deque<ComplianceSummary> kept;
  for (ComplianceSummary& summary : open_summaries_)
  {
    if (time >= summary.window_end)
    {
      kept.push_back(summary);
      continue;
    }
    if (summary.was_compliant)
    {
      summary.end_reason = ComplianceSummary::END_CANCELLED;
    }
    else
    {
      summary.not_compliant_reason = ComplianceSummary::NOT_COMPLIANT_CANCELLED;
    }
    summary.header.stamp = time;
    closed_summaries_.push_back(summary);
  }
  open_summaries_.swap(kept);
}

void ComplianceScheduler::closeFrontSummary(const ros::Time& time)
{
  open_summaries_.front().header.stamp = time;
  closed_summaries_.push_back(open_summaries_.front());
  open_summaries_.pop_front();
}

ComplianceSummarySeverity classifyComplianceSummary(const ComplianceSummary& summary)
{
  if (!summary.was_compliant)
  {
    return summary.not_compliant_reason == ComplianceSummary::NOT_COMPLIANT_CANCELLED ?
               ComplianceSummarySeverity::NONE :
               ComplianceSummarySeverity::WARNING;
  }
  if (summary.end_reason == ComplianceSummary::END_WINDOW)
  {
    return ComplianceSummarySeverity::NONE;
  }
  return ComplianceSummarySeverity::WARNING;
}

std::string describeComplianceSummary(const ComplianceSummary& summary)
{
  std::ostringstream description;
  description << std::fixed << std::setprecision(3) << "Compliance window [" << summary.window_begin.toSec() << ", "
              << summary.window_end.toSec() << "] s (profile " << static_cast<int>(summary.profile) << ") ";
  if (!summary.was_compliant)
  {
    description << "never ran: " << describeNotCompliantReason(summary.not_compliant_reason);
    return description.str();
  }
  description << "compliant [" << summary.compliance_begin.toSec() << ", " << summary.compliance_end.toSec()
              << "] s, ended: " << describeEndReason(summary.end_reason);
  return description.str();
}

}  // namespace ur_driver
