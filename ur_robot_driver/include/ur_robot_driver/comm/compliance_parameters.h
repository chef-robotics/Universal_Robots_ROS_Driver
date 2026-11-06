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
  // Both 0: never hand back to servoj early.
  double handback_max_deflection = 0.0;  // [m]
  double handback_max_speed = 0.0;       // [m/s]
};

// The reverse interface sends each value as an int32 of value times this:
// 0.01 resolution over +-21 million, beyond any physical profile.
constexpr int32_t k_compliance_multiplier = 100;
// The hand-back limits are millimetre-scale, so they get 10 um (um/s)
// resolution over +-21 km instead.
constexpr int32_t k_compliance_handback_multiplier = 100000;
// Stiffness, damping, max force, hand-back deflection and speed.
constexpr int k_num_compliance_values = 6 + 6 + 1 + 2;

/*!
 * \brief Whether `value` survives the reverse interface's fixed-point
 * encoding with `multiplier`.
 */
inline bool isComplianceValueRepresentable(double value, int32_t multiplier = k_compliance_multiplier)
{
  const double limit = static_cast<double>(std::numeric_limits<int32_t>::max()) / multiplier;
  return std::isfinite(value) && std::abs(value) <= limit;
}

}  // namespace comm
}  // namespace ur_driver

#endif  // UR_RTDE_DRIVER_COMPLIANCE_PARAMETERS_H_INCLUDED
