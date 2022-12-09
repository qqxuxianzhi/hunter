//
#include "vehicle_model_wrapper.h"
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
#include "hunter/vehicle_model_hunter.h"
#else
Error : Invalid vehicle platform.
#endif

namespace phoenix {
namespace veh_model {

VehicleModelWrapper::VehicleModelWrapper() {}

VehicleModelWrapper::~VehicleModelWrapper() {
  // nothing to do
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetVehicleLength() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->GetVehicleLength();
#else
  Error:
    Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetVehicleWidth() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->GetVehicleWidth();
#else
  Error:
    Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetDistOfLocalizationToFront()
    const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->GetDistOfLocalizationToFront();
#else
  Error:
    Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetDistOfLocalizationToRear()
    const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->GetDistOfLocalizationToRear();
#else
  Error:
    Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetDistOfLocalizationToCenter()
    const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->GetDistOfLocalizationToCenter();
#else
  Error:
    Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetMaxSteeringAngle() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->GetMaxSteeringAngle();
#else
  Error:
    Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetWheelbase() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->GetWheelbase();
#else
  Error:
    Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::ClampMaxSteeringAngle(
    Scalar angle) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->ClampMaxSteeringAngle(angle);
#else
  Error:
    Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar
VehicleModelWrapper::ClampMaxSteeringAngleByVelocity(Scalar angle,
                                                     Scalar v) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->ClampMaxSteeringAngleByVelocity(angle,
                                                                         v);
#else
  Error:
    Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::CalcYawRateFromSteeringAngle(
    Scalar angle, Scalar v) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->CalcYawRateFromSteeringAngle(angle, v);
#else
  Error:
    Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::CalcSteeringAngleFromYawRate(
    Scalar yaw_rate, Scalar v) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->CalcSteeringAngleFromYawRate(yaw_rate,
                                                                      v);
#else
  Error:
    Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::CalcCurvatureFromSteeringAngle(
    Scalar angle) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->CalcCurvatureFromSteeringAngle(angle);
#else
  Error:
    Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::CalcSteeringAngleFromCurvature(
    Scalar curvature) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->CalcSteeringAngleFromCurvature(
      curvature);
#else
  Error:
    Vehicle model have not been defined.
#endif
}

void VehicleModelWrapper::EstimateNextPos(const Scalar v, const Scalar yaw_rate,
                                          const Scalar dt, const Scalar pos[3],
                                          Scalar next_pos[3]) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->EstimateNextPos(v, yaw_rate, dt, pos,
                                                         next_pos);
#else
  Error:
    Vehicle model have not been defined.
#endif
}

const SensorOnVehicle& VehicleModelWrapper::GetSensor() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_HUNTER)
  return VehicleModelHunter::instance()->GetSensor();
#else
  Error:
    Vehicle model have not been defined.
#endif
}

}  // namespace veh_model
}  // namespace phoenix
