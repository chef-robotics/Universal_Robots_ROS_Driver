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
 * \author  Tristan Schnell schnell@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_REVERSE_INTERFACE_H_INCLUDED
#define UR_RTDE_DRIVER_REVERSE_INTERFACE_H_INCLUDED

#include "ur_robot_driver/comm/compliance_parameters.h"
#include "ur_robot_driver/comm/server.h"
#include "ur_robot_driver/types.h"
#include <cstring>
#include <endian.h>

namespace ur_driver
{
namespace comm
{
/*!
 * \brief Control modes as interpreted from the script runnning on the robot.
 */
enum class ControlMode : int32_t
{
  MODE_STOPPED = -2,        ///< When this is set, the program is expected to stop and exit.
  MODE_UNINITIALIZED = -1,  ///< Startup default until another mode is sent to the script.
  MODE_IDLE = 0,            ///< Set when no controller is currently active controlling the robot.
  MODE_SERVOJ = 1,          ///< Set when servoj control is active.
  MODE_SPEEDJ = 2,          ///< Set when speedj control is active.
  MODE_SERVOJ_COMPLIANT = 3 ///< Set when servoj control is active with compliance (robot script on PolyScope >= 5.23).
};

/*!
 * \brief The ReverseInterface class handles communication to the robot. It starts a server and
 * waits for the robot to connect via its URCaps program.
 */
class ReverseInterface
{
public:
  ReverseInterface() = delete;
  /*!
   * \brief Creates a ReverseInterface object including a URServer.
   *
   * \param port Port the Server is started on
   */
  ReverseInterface(uint32_t port) : server_(port)
  {
    if (!server_.bind())
    {
      throw std::runtime_error("Could not bind to server");
    }
    if (!server_.accept())
    {
      throw std::runtime_error("Failed to accept robot connection");
    }
  }
  /*!
   * \brief Disconnects possible clients so the reverse interface object can be safely destroyed.
   */
  ~ReverseInterface()
  {
    server_.disconnectClient();
  }

  /*!
   * \brief Writes needed information to the robot to be read by the URCaps program.
   *
   * \param positions A vector of joint targets for the robot
   * \param control_mode Control mode assigned to this command. See documentation of ::ControlMode
   * for details on possible values.
   * \param compliance Values of the active compliance profile, sent with
   * MODE_SERVOJ_COMPLIANT; zeros are sent when null. Each value must satisfy
   * isComplianceValueRepresentable().
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool write(const vector6d_t* positions, const ControlMode control_mode = ControlMode::MODE_IDLE,
             const ComplianceParameters* compliance = nullptr)
  {
    uint8_t buffer[k_command_packet_size];
    packCommand(buffer, positions, control_mode, compliance);
    size_t written;
    return server_.write(buffer, sizeof(buffer), written);
  }

  // Keepalive, 6 joint values, control mode, compliance values, each a
  // big-endian int32. The robot script reads exactly this many integers.
  static constexpr size_t k_command_packet_size = sizeof(int32_t) * (1 + 6 + 1 + k_num_compliance_values);

  /*!
   * \brief Serializes one command packet into `buffer`
   * (k_command_packet_size bytes); see write() for the arguments.
   * Null positions or compliance are sent as zeros.
   */
  static void packCommand(uint8_t* buffer, const vector6d_t* positions, const ControlMode control_mode,
                          const ComplianceParameters* compliance)
  {
    std::memset(buffer, 0, k_command_packet_size);
    uint8_t* b_pos = buffer;

    // The first element is always the keepalive signal.
    int32_t val = htobe32(1);
    b_pos += append(b_pos, val);

    if (positions != nullptr)
    {
      for (auto const& pos : *positions)
      {
        int32_t val = static_cast<int32_t>(pos * MULT_JOINTSTATE);
        val = htobe32(val);
        b_pos += append(b_pos, val);
      }
    }
    else
    {
      b_pos += 6 * sizeof(int32_t);
    }

    val = htobe32(toUnderlying(control_mode));
    b_pos += append(b_pos, val);

    if (compliance != nullptr)
    {
      for (const double value : compliance->stiffness)
      {
        b_pos += appendComplianceValue(b_pos, value);
      }
      for (const double value : compliance->damping)
      {
        b_pos += appendComplianceValue(b_pos, value);
      }
      b_pos += appendComplianceValue(b_pos, compliance->max_force);
      b_pos += appendComplianceValue(b_pos, compliance->handback_max_deflection, k_compliance_handback_multiplier);
      b_pos += appendComplianceValue(b_pos, compliance->handback_max_speed, k_compliance_handback_multiplier);
    }
  }

  /*!
   * \brief Reads a keepalive signal from the robot.
   *
   * \returns The received keepalive string or the empty string, if nothing was received
   */
  std::string readKeepalive()
  {
    size_t buf_len = 16;
    char buffer[buf_len];

    bool read_successful = server_.readLine(buffer, buf_len);

    if (read_successful)
    {
      return std::string(buffer);
    }
    else
    {
      return std::string("");
    }
  }

private:
  URServer server_;
  static const int32_t MULT_JOINTSTATE = 1000000;

  static size_t appendComplianceValue(uint8_t* buffer, double value, int32_t multiplier = k_compliance_multiplier)
  {
    int32_t val = htobe32(static_cast<int32_t>(std::round(value * multiplier)));
    return append(buffer, val);
  }

  template <typename T>
  static size_t append(uint8_t* buffer, T& val)
  {
    size_t s = sizeof(T);
    std::memcpy(buffer, &val, s);
    return s;
  }
};

}  // namespace comm
}  // namespace ur_driver

#endif  // UR_RTDE_DRIVER_REVERSE_INTERFACE_H_INCLUDED
