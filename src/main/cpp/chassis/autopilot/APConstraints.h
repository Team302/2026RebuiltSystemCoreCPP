#pragma once

#include "wpi/units/acceleration.hpp"
#include "wpi/units/velocity.hpp"

class APConstraints
{
public:
  APConstraints();
  APConstraints(wpi::units::velocity::meters_per_second_t velocity,
                wpi::units::acceleration::meters_per_second_squared_t acceleration,
                double jerkMetersPerSecondCubed);
  APConstraints(wpi::units::acceleration::meters_per_second_squared_t acceleration,
                double jerkMetersPerSecondCubed);

  APConstraints WithVelocity(wpi::units::velocity::meters_per_second_t newVelocity) const;
  APConstraints WithAcceleration(wpi::units::acceleration::meters_per_second_squared_t newAcceleration) const;
  APConstraints WithJerkMetersPerSecondCubed(double newJerkMetersPerSecondCubed) const;

  wpi::units::velocity::meters_per_second_t GetVelocity() const;
  wpi::units::acceleration::meters_per_second_squared_t GetAcceleration() const;
  double GetJerkMetersPerSecondCubed() const;

  wpi::units::velocity::meters_per_second_t CalculateMaxVelocity(double dist) const;

private:
  wpi::units::velocity::meters_per_second_t AccelerationConstrainedVelocity(double dist) const;
  wpi::units::velocity::meters_per_second_t JerkMetersPerSecondCubedConstrainedVelocity(double dist) const;

  wpi::units::velocity::meters_per_second_t m_velocity;
  wpi::units::acceleration::meters_per_second_squared_t m_acceleration;
  double m_jerkMetersPerSecondCubed;
  double m_x0;
  double m_v0;
};
