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
  // Scale on the spring force this cycle, in [0, 1]; see
  // ComplianceProfile::spring_fade_duration.
  double spring_weight = 1.0;
};

// The reverse interface sends each value as an int32 of value times this:
// 0.01 resolution over +-21 million, beyond any physical profile.
constexpr int32_t k_compliance_multiplier = 100;
// The spring weight gets 1e-5 resolution instead.
constexpr int32_t k_compliance_spring_weight_multiplier = 100000;
// Stiffness, damping, max force, spring weight.
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
