#pragma once

#include "wpi/units/acceleration.hpp"
#include "wpi/units/velocity.hpp"

class APConstraints
{
public:
  APConstraints();
  APConstraints(wpi::units::velocity::meters_per_second_t velocity,
                wpi::units::acceleration::meters_per_second_squared_t acceleration,
                double jerk);
  APConstraints(wpi::units::acceleration::meters_per_second_squared_t acceleration,
                double jerk);

  APConstraints WithVelocity(wpi::units::velocity::meters_per_second_t newVelocity) const;
  APConstraints WithAcceleration(wpi::units::acceleration::meters_per_second_squared_t newAcceleration) const;
  APConstraints WithJerk(double newJerk) const;

  wpi::units::velocity::meters_per_second_t GetVelocity() const;
  wpi::units::acceleration::meters_per_second_squared_t GetAcceleration() const;
  double GetJerk() const;

  wpi::units::velocity::meters_per_second_t CalculateMaxVelocity(double dist) const;

private:
  wpi::units::velocity::meters_per_second_t AccelerationConstrainedVelocity(double dist) const;
  wpi::units::velocity::meters_per_second_t JerkConstrainedVelocity(double dist) const;

  wpi::units::velocity::meters_per_second_t velocity_;
  wpi::units::acceleration::meters_per_second_squared_t acceleration_;
  double jerk_;
  double x0_;
  double v0_;
};
