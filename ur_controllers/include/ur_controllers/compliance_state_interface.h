#ifndef UR_CONTROLLERS_COMPLIANCE_STATE_INTERFACE_H_INCLUDED
#define UR_CONTROLLERS_COMPLIANCE_STATE_INTERFACE_H_INCLUDED

#include <string>

#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace ur_controllers
{
/*!
 * \brief Read-only handle to whether the robot script is in compliant (torque) mode.
 */
class ComplianceStateHandle
{
public:
  ComplianceStateHandle() = default;
  ComplianceStateHandle(const std::string& name, const bool* is_compliant) : name_(name), is_compliant_(is_compliant)
  {
    if (is_compliant_ == nullptr)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. Compliance state pointer is null.");
    }
  }

  std::string getName() const
  {
    return name_;
  }

  bool isCompliant() const
  {
    assert(is_compliant_);
    return *is_compliant_;
  }

private:
  std::string name_;
  const bool* is_compliant_ = nullptr;
};

/*! \brief Hardware interface for reading whether the robot script is compliant; claims nothing. */
class ComplianceStateInterface : public hardware_interface::HardwareResourceManager<ComplianceStateHandle>
{
};
}  // namespace ur_controllers

#endif  // ifndef UR_CONTROLLERS_COMPLIANCE_STATE_INTERFACE_H_INCLUDED
