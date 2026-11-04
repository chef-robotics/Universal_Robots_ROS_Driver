#ifndef UR_RTDE_DRIVER_COMPLIANCE_PARAMETERS_H_INCLUDED
#define UR_RTDE_DRIVER_COMPLIANCE_PARAMETERS_H_INCLUDED

#include <cmath>
#include <cstdint>
#include <limits>

#include "ur_robot_driver/types.h"

namespace ur_driver
{
namespace comm
{
/*!
 * \brief Values of the active compliance profile, sent to the robot script
 * next to `MODE_SERVOJ_COMPLIANT` (see ur_extra_msgs/ComplianceProfile).
 */
struct ComplianceParameters
{
  vector6d_t stiffness;  // Tool frame [N/m, N·m/rad].
  vector6d_t damping;    // Tool frame [N·s/m, N·m·s/rad].
  double max_force;      // [N]
  double max_torque;     // [N·m]
};

// The reverse interface sends each value as an int32 of value times this.
// 1e4 covers stiffnesses of tens of thousands of N/m at 1e-4 resolution.
constexpr int32_t k_compliance_multiplier = 10000;
// Stiffness, damping, max force, max torque.
constexpr int k_num_compliance_values = 6 + 6 + 1 + 1;

/*!
 * \brief Whether `value` survives the reverse interface's fixed-point encoding.
 */
inline bool isComplianceValueRepresentable(double value)
{
  const double limit = static_cast<double>(std::numeric_limits<int32_t>::max()) / k_compliance_multiplier;
  return std::isfinite(value) && std::abs(value) <= limit;
}

}  // namespace comm
}  // namespace ur_driver

#endif  // UR_RTDE_DRIVER_COMPLIANCE_PARAMETERS_H_INCLUDED
