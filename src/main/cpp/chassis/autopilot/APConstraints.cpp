#include "APConstraints.h"

#include <cmath>
#include <limits>

using wpi::units::acceleration::meters_per_second_squared_t;
using wpi::units::velocity::meters_per_second_t;

namespace
{
  const meters_per_second_t kInfinityVelocity{std::numeric_limits<double>::infinity()};
  const meters_per_second_squared_t kInfinityAcceleration{std::numeric_limits<double>::infinity()};
  constexpr double kInfinityJerkMetersPerSecondCubed = std::numeric_limits<double>::infinity();
  constexpr double kAccelerationDenominator = 18.0;
  constexpr double kAccelerationMultiplier = 2.0;
  constexpr double kJerkMetersPerSecondCubedMultiplier = 4.5;
  constexpr double kSquareExponent = 2.0;
  constexpr double kCubeExponent = 3.0;
  constexpr double kCubicRootExponent = 1.0 / 3.0;
} // namespace

APConstraints::APConstraints()
    : APConstraints(kInfinityVelocity, kInfinityAcceleration, kInfinityJerkMetersPerSecondCubed) {}

APConstraints::APConstraints(meters_per_second_t velocity,
                             meters_per_second_squared_t acceleration,
                             double jerkMetersPerSecondCubed)
    : m_velocity(velocity),
      m_acceleration(acceleration),
      m_jerkMetersPerSecondCubed(jerkMetersPerSecondCubed),
      m_x0(std::pow(acceleration.value(), kCubeExponent) /
           (kAccelerationDenominator * jerkMetersPerSecondCubed * jerkMetersPerSecondCubed)),
      m_v0(JerkMetersPerSecondCubedConstrainedVelocity(m_x0).value()) {}

APConstraints::APConstraints(meters_per_second_squared_t acceleration, double jerkMetersPerSecondCubed)
    : APConstraints(kInfinityVelocity, acceleration, jerkMetersPerSecondCubed) {}

APConstraints APConstraints::WithVelocity(meters_per_second_t newVelocity) const
{
  return APConstraints(newVelocity, m_acceleration, m_jerkMetersPerSecondCubed);
}

APConstraints APConstraints::WithAcceleration(meters_per_second_squared_t newAcceleration) const
{
  return APConstraints(m_velocity, newAcceleration, m_jerkMetersPerSecondCubed);
}

APConstraints APConstraints::WithJerkMetersPerSecondCubed(double newJerkMetersPerSecondCubed) const
{
  return APConstraints(m_velocity, m_acceleration, newJerkMetersPerSecondCubed);
}

meters_per_second_t APConstraints::GetVelocity() const
{
  return m_velocity;
}

meters_per_second_squared_t APConstraints::GetAcceleration() const
{
  return m_acceleration;
}

double APConstraints::GetJerkMetersPerSecondCubed() const
{
  return m_jerkMetersPerSecondCubed;
}

meters_per_second_t APConstraints::CalculateMaxVelocity(double dist) const
{
  if (dist > m_x0)
  {
    return AccelerationConstrainedVelocity(dist);
  }
  return JerkMetersPerSecondCubedConstrainedVelocity(dist);
}

meters_per_second_t APConstraints::AccelerationConstrainedVelocity(double dist) const
{
  return meters_per_second_t{
      std::sqrt(m_v0 * m_v0 + kAccelerationMultiplier * m_acceleration.value() * (dist - m_x0))};
}

meters_per_second_t APConstraints::JerkMetersPerSecondCubedConstrainedVelocity(double dist) const
{
  return meters_per_second_t{
      std::pow((kJerkMetersPerSecondCubedMultiplier * std::pow(dist, kSquareExponent)) * m_jerkMetersPerSecondCubed,
               kCubicRootExponent)};
}
