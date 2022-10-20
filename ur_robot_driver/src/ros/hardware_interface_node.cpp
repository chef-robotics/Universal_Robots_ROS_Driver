// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <csignal>
#include <ur_robot_driver/ros/hardware_interface.h>

std::unique_ptr<ur_driver::HardwareInterface> g_hw_interface;

void signalHandler(int signum)
{
  std::cout << "Interrupt signal (" << signum << ") received.\n";

  g_hw_interface.reset();
  // cleanup and close up stuff here
  // terminate program

  exit(signum);
}

struct msgStats {
  std::chrono::duration<double> min_stat;
  std::chrono::duration<double> max_stat;
  std::chrono::duration<double> total_stat;
};

void reset_msg_stats(msgStats &stats)
{
  stats.min_stat = std::chrono::duration<double>::zero();
  stats.max_stat = std::chrono::duration<double>::zero();
  stats.total_stat = std::chrono::duration<double>::zero();
}

void update_msg_stats(std::chrono::duration<double> stat_elapsed,
                      msgStats &stats)
{
  if (stats.min_stat == std::chrono::duration<double>::zero() || stat_elapsed < stats.min_stat) stats.min_stat = stat_elapsed;
  if (stats.max_stat == std::chrono::duration<double>::zero() || stat_elapsed > stats.max_stat) stats.max_stat = stat_elapsed;
  stats.total_stat += stat_elapsed;
}

void push_msg_stats(diagnostic_msgs::DiagnosticStatus &stat, 
                    std::string stat_name, 
                    msgStats msg_stats,
                    long num_loops)
{
  diagnostic_msgs::KeyValue min_stat_kv;
  min_stat_kv.key = "min " + stat_name + " (s)";
  min_stat_kv.value = std::to_string(msg_stats.min_stat.count());
  stat.values.push_back(min_stat_kv);
  diagnostic_msgs::KeyValue max_stat_kv;
  max_stat_kv.key = "max " + stat_name + " (s)";
  max_stat_kv.value = std::to_string(msg_stats.max_stat.count());
  stat.values.push_back(max_stat_kv);
  diagnostic_msgs::KeyValue avg_stat_kv;
  avg_stat_kv.key = "avg " + stat_name + " (s)";
  avg_stat_kv.value = std::to_string(msg_stats.total_stat.count() / num_loops);
  stat.values.push_back(avg_stat_kv);
}

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ur_hardware_interface");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  ros::Publisher diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
  bool has_realtime;
  realtime_file >> has_realtime;
  if (has_realtime)
  {
    const int max_thread_priority = sched_get_priority_max(SCHED_FIFO);
    if (max_thread_priority != -1)
    {
      // We'll operate on the currently running thread.
      pthread_t this_thread = pthread_self();

      // struct sched_param is used to store the scheduling priority
      struct sched_param params;

      // We'll set the priority to the maximum.
      params.sched_priority = max_thread_priority;

      int ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
      if (ret != 0)
      {
        ROS_ERROR_STREAM("Unsuccessful in setting main thread realtime priority. Error code: " << ret);
      }
      // Now verify the change in thread priority
      int policy = 0;
      ret = pthread_getschedparam(this_thread, &policy, &params);
      if (ret != 0)
      {
        std::cout << "Couldn't retrieve real-time scheduling paramers" << std::endl;
      }

      // Check the correct policy was applied
      if (policy != SCHED_FIFO)
      {
        ROS_ERROR("Main thread: Scheduling is NOT SCHED_FIFO!");
      }
      else
      {
        ROS_INFO("Main thread: SCHED_FIFO OK");
      }

      // Print thread scheduling priority
      ROS_INFO_STREAM("Main thread priority is " << params.sched_priority);
    }
    else
    {
      ROS_ERROR("Could not get maximum thread priority for main thread");
    }
  }

  // Set up timers
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  g_hw_interface.reset(new ur_driver::HardwareInterface);

  if (!g_hw_interface->init(nh, nh_priv))
  {
    ROS_ERROR_STREAM("Could not correctly initialize robot. Exiting");
    exit(1);
  }
  ROS_DEBUG_STREAM("initialized hw interface");
  controller_manager::ControllerManager cm(g_hw_interface.get(), nh);

  // Get current time and elapsed time since last read
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  double expected_cycle_time = 1.0 / (static_cast<double>(g_hw_interface->getControlFrequency()));

  msgStats period_stats;
  reset_msg_stats(period_stats);
  msgStats read_stats;
  reset_msg_stats(read_stats);
  msgStats cm_update_stats;
  reset_msg_stats(cm_update_stats);
  msgStats write_stats;
  reset_msg_stats(write_stats);

  // sub-stats of the read method
  msgStats get_data_stats;
  reset_msg_stats(get_data_stats);
  msgStats read_data_stats;
  reset_msg_stats(read_data_stats);
  msgStats pub_io_stats;
  reset_msg_stats(pub_io_stats);
  msgStats pub_tool_stats;
  reset_msg_stats(pub_tool_stats);
  msgStats pub_pose_stats;
  reset_msg_stats(pub_pose_stats);
  msgStats pub_robot_stats;
  reset_msg_stats(pub_robot_stats);
  msgStats pub_temp_stats;
  reset_msg_stats(pub_temp_stats);

  std::chrono::duration<double> last_diagnostics_duration = std::chrono::duration<double>::zero();

  long debug_loops = 0;

  // Debug timing printout every 5 seconds
  const std::chrono::seconds debug_timing_period{5};
  std::chrono::steady_clock::time_point debug_timing_start = std::chrono::steady_clock::now();

  // Run as fast as possible
  while (ros::ok())
  {
    const std::chrono::steady_clock::time_point debug_timing_now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed_since_debug = debug_timing_now - debug_timing_start;
    // This is mostly used to track low-frequency information like joint temperature
    const bool trigger_low_frequency_logging = elapsed_since_debug > debug_timing_period;
    g_hw_interface->shouldLogTemperature(trigger_low_frequency_logging);

    // Receive current state from robot
    const std::chrono::steady_clock::time_point read_start = std::chrono::steady_clock::now();
    g_hw_interface->read(timestamp, period);
    const std::chrono::duration<double> read_elapsed = std::chrono::steady_clock::now() - read_start;
    update_msg_stats(read_elapsed, read_stats);
    ur_driver::HardwareInterface::readStats read_substats = g_hw_interface->get_read_stats();
    update_msg_stats(read_substats.get_data_elapsed, get_data_stats);
    update_msg_stats(read_substats.read_data_elapsed, read_data_stats);
    update_msg_stats(read_substats.pub_io_elapsed, pub_io_stats);
    update_msg_stats(read_substats.pub_tool_elapsed, pub_tool_stats);
    update_msg_stats(read_substats.pub_pose_elapsed, pub_pose_stats);
    update_msg_stats(read_substats.pub_robot_elapsed, pub_robot_stats);
    update_msg_stats(read_substats.pub_temp_elapsed, pub_temp_stats);


    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> period_chrono = stopwatch_now - stopwatch_last;
    update_msg_stats(period_chrono, period_stats);
    period.fromSec(period_chrono.count());
    stopwatch_last = stopwatch_now;

    const std::chrono::steady_clock::time_point cm_update_start = std::chrono::steady_clock::now();
    cm.update(timestamp, period, g_hw_interface->shouldResetControllers());
    const std::chrono::duration<double> cm_update_elapsed = std::chrono::steady_clock::now() - cm_update_start;
    update_msg_stats(cm_update_elapsed, cm_update_stats);

    const std::chrono::steady_clock::time_point write_start = std::chrono::steady_clock::now();
    g_hw_interface->write(timestamp, period);
    const std::chrono::duration<double> write_elapsed = std::chrono::steady_clock::now() - write_start;
    update_msg_stats(write_elapsed, write_stats);

    ++debug_loops;

    // Check if it's time to print
    if (trigger_low_frequency_logging) {
      const std::chrono::steady_clock::time_point diagnostic_start = std::chrono::steady_clock::now();
      diagnostic_msgs::DiagnosticArray dia_array;
      diagnostic_msgs::DiagnosticStatus robot_status;
      robot_status.name = "ur_hardware_interface: Overall health";
      robot_status.level = diagnostic_msgs::DiagnosticStatus::OK;
      robot_status.message = "Looping";
      robot_status.hardware_id = "none";

      diagnostic_msgs::KeyValue loop_durations_last;
      loop_durations_last.key = "Loop durations last (s)";
      loop_durations_last.value = std::to_string(elapsed_since_debug.count());
      robot_status.values.push_back(loop_durations_last);
      diagnostic_msgs::KeyValue num_loops;
      num_loops.key = "Number of loops";
      num_loops.value = std::to_string(debug_loops);
      robot_status.values.push_back(num_loops);

      push_msg_stats(robot_status, "period loop", period_stats, debug_loops);
      push_msg_stats(robot_status, "read", read_stats, debug_loops);
      push_msg_stats(robot_status, "cm_update", cm_update_stats, debug_loops);
      push_msg_stats(robot_status, "write", write_stats, debug_loops);
      push_msg_stats(robot_status, "read: get_data", get_data_stats, debug_loops);
      push_msg_stats(robot_status, "read: read_data", read_data_stats, debug_loops);
      push_msg_stats(robot_status, "read: pub_io", pub_io_stats, debug_loops);
      push_msg_stats(robot_status, "read: pub_tool", pub_tool_stats, debug_loops);
      push_msg_stats(robot_status, "read: pub_pose", pub_pose_stats, debug_loops);
      push_msg_stats(robot_status, "read: pub_robot", pub_robot_stats, debug_loops);
      push_msg_stats(robot_status, "read: pub_temp", pub_temp_stats, debug_loops);

      diagnostic_msgs::KeyValue diagnostic_duration_kv;
      diagnostic_duration_kv.key = "Last diagnostic duration (s)";
      diagnostic_duration_kv.value = std::to_string(last_diagnostics_duration.count());
      robot_status.values.push_back(diagnostic_duration_kv);

      dia_array.status.push_back(robot_status);
      diagnostic_pub.publish(dia_array);

      reset_msg_stats(period_stats);
      reset_msg_stats(read_stats);
      reset_msg_stats(cm_update_stats);
      reset_msg_stats(write_stats);
      reset_msg_stats(get_data_stats);
      reset_msg_stats(read_data_stats);
      reset_msg_stats(pub_io_stats);
      reset_msg_stats(pub_tool_stats);
      reset_msg_stats(pub_pose_stats);
      reset_msg_stats(pub_robot_stats);
      reset_msg_stats(pub_temp_stats);
      debug_loops = 0;
      last_diagnostics_duration = std::chrono::steady_clock::now() - diagnostic_start;
    }
    if (trigger_low_frequency_logging) debug_timing_start = debug_timing_now;

    // if (!control_rate.sleep())
    // if (period.toSec() > expected_cycle_time)
    // {
      // ROS_WARN_STREAM("Could not keep cycle rate of " << expected_cycle_time * 1000 << "ms");
      // ROS_WARN_STREAM("Actual cycle time:" << period.toNSec() / 1000000.0 << "ms");
    // }
  }

  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");
  return 0;
}
