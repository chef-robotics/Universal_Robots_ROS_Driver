#include <endian.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <ur_robot_driver/comm/reverse_interface.h>
#include <ur_robot_driver/ros/compliance_scheduler.h>

using ur_driver::ComplianceScheduler;
using ur_driver::ComplianceScriptState;
using ur_driver::ComplianceScriptStatus;
using ur_driver::ComplianceSummarySeverity;
using ur_extra_msgs::ComplianceProfile;
using ur_extra_msgs::ComplianceSchedule;
using ur_extra_msgs::ComplianceSummary;
using ur_extra_msgs::ComplianceWindow;

namespace
{
const ros::Time k_stamp(100.0);

ComplianceProfile makeProfile(double stiffness_z)
{
  ComplianceProfile profile;
  profile.stiffness = { { 20000.0, 20000.0, stiffness_z, 600.0, 600.0, 600.0 } };
  profile.damping = { { 630.0, 630.0, 490.0, 11.0, 11.0, 11.0 } };
  profile.max_force = 250.0;
  profile.max_torque = 15.0;
  return profile;
}

ComplianceWindow makeWindow(double begin, double end, uint8_t profile = 0)
{
  ComplianceWindow window;
  window.begin = ros::Duration(begin);
  window.end = ros::Duration(end);
  window.profile = profile;
  return window;
}

// One window [k_stamp + 1.0, k_stamp + 1.5) with one profile.
ComplianceSchedule makeSchedule()
{
  ComplianceSchedule schedule;
  schedule.stamp = k_stamp;
  schedule.profiles.push_back(makeProfile(12000.0));
  schedule.windows.push_back(makeWindow(1.0, 1.5));
  return schedule;
}

ros::Time at(double seconds_after_stamp)
{
  return k_stamp + ros::Duration(seconds_after_stamp);
}

ComplianceScriptStatus makeStatus(ComplianceScriptState state, double deflection_z = 0.0)
{
  ComplianceScriptStatus status;
  status.state = static_cast<int32_t>(state);
  status.deflection = { { 0.003, 0.004, deflection_z } };
  return status;
}

void expectRejected(const ComplianceSchedule& schedule)
{
  ComplianceScheduler scheduler;
  scheduler.setSchedule(makeSchedule(), at(0.0));
  EXPECT_THROW(scheduler.setSchedule(schedule, at(0.0)), std::invalid_argument);
  // The previous schedule is unchanged.
  EXPECT_NE(scheduler.findActiveParameters(at(1.2)), nullptr);
  EXPECT_TRUE(scheduler.popClosedSummaries().empty());
}

int32_t extractPacketInt(const uint8_t* buffer, size_t idx)
{
  int32_t value;
  std::memcpy(&value, buffer + idx * sizeof(int32_t), sizeof(int32_t));
  return static_cast<int32_t>(be32toh(value));
}

}  // namespace

TEST(ComplianceScheduler, activeOnlyInsideWindow)
{
  ComplianceScheduler scheduler;
  EXPECT_EQ(scheduler.findActiveParameters(at(1.2)), nullptr);
  scheduler.setSchedule(makeSchedule(), at(0.0));
  EXPECT_EQ(scheduler.findActiveParameters(at(0.999)), nullptr);
  const ur_driver::comm::ComplianceParameters* parameters = scheduler.findActiveParameters(at(1.0));
  ASSERT_NE(parameters, nullptr);
  EXPECT_EQ(parameters->stiffness[2], 12000.0);
  EXPECT_EQ(parameters->damping[3], 11.0);
  EXPECT_EQ(parameters->max_force, 250.0);
  EXPECT_EQ(parameters->max_torque, 15.0);
  EXPECT_NE(scheduler.findActiveParameters(at(1.499)), nullptr);
  EXPECT_EQ(scheduler.findActiveParameters(at(1.5)), nullptr);
}

TEST(ComplianceScheduler, eachWindowUsesItsProfile)
{
  ComplianceSchedule schedule = makeSchedule();
  schedule.profiles.push_back(makeProfile(3000.0));
  schedule.windows.push_back(makeWindow(2.0, 2.5, 1));
  ComplianceScheduler scheduler;
  scheduler.setSchedule(schedule, at(0.0));
  EXPECT_EQ(scheduler.findActiveParameters(at(1.2))->stiffness[2], 12000.0);
  EXPECT_EQ(scheduler.findActiveParameters(at(2.2))->stiffness[2], 3000.0);
}

TEST(ComplianceScheduler, emptyScheduleClears)
{
  ComplianceScheduler scheduler;
  scheduler.setSchedule(makeSchedule(), at(0.0));
  scheduler.setSchedule(ComplianceSchedule(), at(0.0));
  EXPECT_EQ(scheduler.findActiveParameters(at(1.2)), nullptr);
}

TEST(ComplianceScheduler, rejectsInvalidWindows)
{
  ComplianceSchedule empty_window = makeSchedule();
  empty_window.windows[0] = makeWindow(1.0, 1.0);
  expectRejected(empty_window);

  ComplianceSchedule negative_begin = makeSchedule();
  negative_begin.windows[0] = makeWindow(-0.1, 1.0);
  expectRejected(negative_begin);

  ComplianceSchedule overlapping = makeSchedule();
  overlapping.windows.push_back(makeWindow(1.4, 2.0));
  expectRejected(overlapping);

  ComplianceSchedule out_of_order = makeSchedule();
  out_of_order.windows.insert(out_of_order.windows.begin(), makeWindow(2.0, 2.5));
  expectRejected(out_of_order);

  ComplianceSchedule missing_profile = makeSchedule();
  missing_profile.windows[0].profile = 1;
  expectRejected(missing_profile);

  ComplianceSchedule unstamped = makeSchedule();
  unstamped.stamp = ros::Time();
  expectRejected(unstamped);
}

TEST(ComplianceScheduler, rejectsUnrepresentableProfileValues)
{
  ComplianceSchedule not_finite = makeSchedule();
  not_finite.profiles[0].stiffness[0] = std::numeric_limits<double>::quiet_NaN();
  expectRejected(not_finite);

  ComplianceSchedule too_large = makeSchedule();
  too_large.profiles[0].damping[1] = 1e6;
  expectRejected(too_large);
}

TEST(ComplianceScheduler, summarizesWindowEnd)
{
  ComplianceScheduler scheduler;
  scheduler.setSchedule(makeSchedule(), at(0.0));
  const ComplianceScriptStatus servoj = makeStatus(ComplianceScriptState::SERVOJ);
  scheduler.updateSummaries(at(0.5), servoj);
  scheduler.updateSummaries(at(1.0), servoj);
  ComplianceScriptStatus compliant = makeStatus(ComplianceScriptState::COMPLIANT, 0.030);
  compliant.force = 80.0;
  scheduler.updateSummaries(at(1.002), compliant);
  compliant.deflection[2] = 0.010;
  compliant.force = 40.0;
  // The script leaves compliance a few cycles after the window end.
  scheduler.updateSummaries(at(1.55), compliant);
  EXPECT_TRUE(scheduler.popClosedSummaries().empty());
  scheduler.updateSummaries(at(1.6), makeStatus(ComplianceScriptState::SERVOJ));

  const std::vector<ComplianceSummary> summaries = scheduler.popClosedSummaries();
  ASSERT_EQ(summaries.size(), 1u);
  const ComplianceSummary& summary = summaries[0];
  EXPECT_EQ(summary.header.stamp, at(1.6));
  EXPECT_EQ(summary.window_begin, at(1.0));
  EXPECT_EQ(summary.window_end, at(1.5));
  EXPECT_EQ(summary.profile, 0);
  EXPECT_TRUE(summary.was_compliant);
  EXPECT_EQ(summary.not_compliant_reason, ComplianceSummary::NOT_COMPLIANT_NONE);
  EXPECT_EQ(summary.compliance_begin, at(1.002));
  EXPECT_EQ(summary.compliance_end, at(1.55));
  EXPECT_EQ(summary.end_reason, ComplianceSummary::END_WINDOW);
  EXPECT_EQ(summary.max_force, 80.0);
  EXPECT_DOUBLE_EQ(summary.max_deflection_z, 0.030);
  EXPECT_DOUBLE_EQ(summary.max_deflection_xy, 0.005);
  EXPECT_DOUBLE_EQ(summary.exit_deflection_z, 0.010);
  EXPECT_EQ(ur_driver::classifyComplianceSummary(summary), ComplianceSummarySeverity::NONE);
  EXPECT_TRUE(scheduler.popClosedSummaries().empty());
}

TEST(ComplianceScheduler, warnsWhenComplianceEndsEarly)
{
  ComplianceScheduler scheduler;
  scheduler.setSchedule(makeSchedule(), at(0.0));
  scheduler.updateSummaries(at(1.1), makeStatus(ComplianceScriptState::COMPLIANT));
  scheduler.updateSummaries(at(1.3), makeStatus(ComplianceScriptState::SERVOJ));
  const std::vector<ComplianceSummary> summaries = scheduler.popClosedSummaries();
  ASSERT_EQ(summaries.size(), 1u);
  EXPECT_EQ(summaries[0].end_reason, ComplianceSummary::END_EARLY);
  EXPECT_EQ(summaries[0].compliance_end, at(1.1));
  EXPECT_EQ(ur_driver::classifyComplianceSummary(summaries[0]), ComplianceSummarySeverity::WARNING);
  EXPECT_NE(ur_driver::describeComplianceSummary(summaries[0]).find("before the window ended"), std::string::npos);
  // The window itself still runs to its end.
  EXPECT_NE(scheduler.findActiveParameters(at(1.4)), nullptr);
}

TEST(ComplianceScheduler, warnsWhenScriptNeverReportsCompliant)
{
  ComplianceScheduler scheduler;
  scheduler.setSchedule(makeSchedule(), at(0.0));
  const ComplianceScriptStatus servoj = makeStatus(ComplianceScriptState::SERVOJ);
  scheduler.updateSummaries(at(1.2), servoj);
  EXPECT_TRUE(scheduler.popClosedSummaries().empty());
  scheduler.updateSummaries(at(1.5), servoj);
  const std::vector<ComplianceSummary> summaries = scheduler.popClosedSummaries();
  ASSERT_EQ(summaries.size(), 1u);
  EXPECT_FALSE(summaries[0].was_compliant);
  EXPECT_EQ(summaries[0].not_compliant_reason, ComplianceSummary::NOT_COMPLIANT_NOT_REPORTED);
  EXPECT_EQ(ur_driver::classifyComplianceSummary(summaries[0]), ComplianceSummarySeverity::WARNING);
}

TEST(ComplianceScheduler, closesWhenScriptStaysCompliant)
{
  ComplianceScheduler scheduler;
  scheduler.setSchedule(makeSchedule(), at(0.0));
  const ComplianceScriptStatus compliant = makeStatus(ComplianceScriptState::COMPLIANT);
  scheduler.updateSummaries(at(1.1), compliant);
  scheduler.updateSummaries(at(1.99), compliant);
  EXPECT_TRUE(scheduler.popClosedSummaries().empty());
  scheduler.updateSummaries(at(2.0), compliant);
  const std::vector<ComplianceSummary> summaries = scheduler.popClosedSummaries();
  ASSERT_EQ(summaries.size(), 1u);
  EXPECT_EQ(summaries[0].end_reason, ComplianceSummary::END_NOT_REPORTED);
  EXPECT_EQ(ur_driver::classifyComplianceSummary(summaries[0]), ComplianceSummarySeverity::WARNING);
}

TEST(ComplianceScheduler, cancelsPendingWindowQuietly)
{
  ComplianceScheduler scheduler;
  scheduler.setSchedule(makeSchedule(), at(0.0));
  ComplianceSchedule replacement = makeSchedule();
  replacement.windows[0] = makeWindow(2.0, 2.5);
  scheduler.setSchedule(replacement, at(0.5));
  const std::vector<ComplianceSummary> summaries = scheduler.popClosedSummaries();
  ASSERT_EQ(summaries.size(), 1u);
  EXPECT_EQ(summaries[0].header.stamp, at(0.5));
  EXPECT_FALSE(summaries[0].was_compliant);
  EXPECT_EQ(summaries[0].not_compliant_reason, ComplianceSummary::NOT_COMPLIANT_CANCELLED);
  EXPECT_EQ(ur_driver::classifyComplianceSummary(summaries[0]), ComplianceSummarySeverity::NONE);
  EXPECT_EQ(scheduler.findActiveParameters(at(1.2)), nullptr);
  EXPECT_NE(scheduler.findActiveParameters(at(2.2)), nullptr);
}

TEST(ComplianceScheduler, warnsOnCancelMidWindow)
{
  ComplianceScheduler scheduler;
  scheduler.setSchedule(makeSchedule(), at(0.0));
  scheduler.updateSummaries(at(1.1), makeStatus(ComplianceScriptState::COMPLIANT));
  scheduler.clearSchedule(at(1.2));
  EXPECT_EQ(scheduler.findActiveParameters(at(1.2)), nullptr);
  const std::vector<ComplianceSummary> summaries = scheduler.popClosedSummaries();
  ASSERT_EQ(summaries.size(), 1u);
  EXPECT_TRUE(summaries[0].was_compliant);
  EXPECT_EQ(summaries[0].end_reason, ComplianceSummary::END_CANCELLED);
  EXPECT_EQ(summaries[0].compliance_end, at(1.1));
  EXPECT_EQ(ur_driver::classifyComplianceSummary(summaries[0]), ComplianceSummarySeverity::WARNING);
}

TEST(ComplianceScheduler, keepsEndedWindowOpenAcrossReplacement)
{
  ComplianceScheduler scheduler;
  scheduler.setSchedule(makeSchedule(), at(0.0));
  const ComplianceScriptStatus compliant = makeStatus(ComplianceScriptState::COMPLIANT);
  scheduler.updateSummaries(at(1.1), compliant);
  scheduler.updateSummaries(at(1.52), compliant);
  // A new schedule arrives before the script has left compliance.
  scheduler.clearSchedule(at(1.53));
  EXPECT_TRUE(scheduler.popClosedSummaries().empty());
  scheduler.updateSummaries(at(1.6), makeStatus(ComplianceScriptState::SERVOJ));
  const std::vector<ComplianceSummary> summaries = scheduler.popClosedSummaries();
  ASSERT_EQ(summaries.size(), 1u);
  EXPECT_EQ(summaries[0].end_reason, ComplianceSummary::END_WINDOW);
}

TEST(ComplianceScheduler, backToBackWindowsSwitchProfileWithoutGap)
{
  // E.g. a grasp window then a lift window with a stiffer profile.
  ComplianceSchedule schedule = makeSchedule();
  schedule.profiles.push_back(makeProfile(3000.0));
  schedule.windows.push_back(makeWindow(1.5, 2.0, 1));
  ComplianceScheduler scheduler;
  scheduler.setSchedule(schedule, at(0.0));
  EXPECT_EQ(scheduler.findActiveParameters(at(1.499))->stiffness[2], 12000.0);
  EXPECT_EQ(scheduler.findActiveParameters(at(1.5))->stiffness[2], 3000.0);

  const ComplianceScriptStatus compliant = makeStatus(ComplianceScriptState::COMPLIANT);
  scheduler.updateSummaries(at(1.4), compliant);
  scheduler.updateSummaries(at(1.5), compliant);
  const std::vector<ComplianceSummary> first = scheduler.popClosedSummaries();
  ASSERT_EQ(first.size(), 1u);
  EXPECT_EQ(first[0].window_end, at(1.5));
  EXPECT_EQ(first[0].end_reason, ComplianceSummary::END_WINDOW);
  scheduler.updateSummaries(at(2.1), makeStatus(ComplianceScriptState::SERVOJ));
  const std::vector<ComplianceSummary> second = scheduler.popClosedSummaries();
  ASSERT_EQ(second.size(), 1u);
  EXPECT_EQ(second[0].profile, 1);
  EXPECT_EQ(second[0].compliance_begin, at(1.5));
  EXPECT_EQ(second[0].end_reason, ComplianceSummary::END_WINDOW);
}

TEST(ComplianceScheduler, peakForceCountsBeforeCompliance)
{
  ComplianceScheduler scheduler;
  scheduler.setSchedule(makeSchedule(), at(0.0));
  ComplianceScriptStatus servoj = makeStatus(ComplianceScriptState::SERVOJ);
  servoj.force = 300.0;
  // Before the window: not counted.
  scheduler.updateSummaries(at(0.9), servoj);
  servoj.force = 120.0;
  scheduler.updateSummaries(at(1.1), servoj);
  scheduler.updateSummaries(at(1.5), servoj);
  const std::vector<ComplianceSummary> summaries = scheduler.popClosedSummaries();
  ASSERT_EQ(summaries.size(), 1u);
  EXPECT_EQ(summaries[0].max_force, 120.0);
}

TEST(ReverseInterface, packsComplianceAfterMode)
{
  using ur_driver::comm::ReverseInterface;
  const size_t packet_size = ReverseInterface::k_command_packet_size;
  ASSERT_EQ(packet_size, 22 * sizeof(int32_t));
  ur_driver::comm::ComplianceParameters compliance;
  compliance.stiffness = { { 20000.0, 20000.0, 12000.0, 600.0, 600.0, 600.0 } };
  compliance.damping = { { 630.0, 630.0, 490.0, 11.0, 11.0, 11.0 } };
  compliance.max_force = 250.0;
  compliance.max_torque = -0.00026;
  const ur_driver::vector6d_t positions = { { 0.1, -0.2, 0.3, -0.4, 0.5, -0.6 } };
  std::vector<uint8_t> buffer(packet_size);
  ReverseInterface::packCommand(buffer.data(), &positions, ur_driver::comm::ControlMode::MODE_SERVOJ_COMPLIANT,
                                &compliance);
  EXPECT_EQ(extractPacketInt(buffer.data(), 0), 1);
  EXPECT_EQ(extractPacketInt(buffer.data(), 2), -200000);
  EXPECT_EQ(extractPacketInt(buffer.data(), 7), 3);
  EXPECT_EQ(extractPacketInt(buffer.data(), 8), 200000000);
  EXPECT_EQ(extractPacketInt(buffer.data(), 10), 120000000);
  EXPECT_EQ(extractPacketInt(buffer.data(), 14), 6300000);
  EXPECT_EQ(extractPacketInt(buffer.data(), 19), 110000);
  EXPECT_EQ(extractPacketInt(buffer.data(), 20), 2500000);
  EXPECT_EQ(extractPacketInt(buffer.data(), 21), -3);

  ReverseInterface::packCommand(buffer.data(), &positions, ur_driver::comm::ControlMode::MODE_SERVOJ, nullptr);
  EXPECT_EQ(extractPacketInt(buffer.data(), 7), 1);
  for (size_t i = 8; i < 22; ++i)
  {
    EXPECT_EQ(extractPacketInt(buffer.data(), i), 0) << i;
  }
}

TEST(ComplianceParameters, representableRange)
{
  EXPECT_TRUE(ur_driver::comm::isComplianceValueRepresentable(214748.0));
  EXPECT_TRUE(ur_driver::comm::isComplianceValueRepresentable(-214748.0));
  EXPECT_FALSE(ur_driver::comm::isComplianceValueRepresentable(214749.0));
  EXPECT_FALSE(ur_driver::comm::isComplianceValueRepresentable(std::numeric_limits<double>::infinity()));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
