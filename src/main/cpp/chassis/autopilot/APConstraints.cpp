#include "APConstraints.h"

#include <cmath>
#include <limits>

using wpi::units::acceleration::meters_per_second_squared_t;
using wpi::units::velocity::meters_per_second_t;

namespace
{
  const meters_per_second_t kInfinityVelocity{std::numeric_limits<double>::infinity()};
  const meters_per_second_squared_t kInfinityAcceleration{std::numeric_limits<double>::infinity()};
  constexpr double kInfinityJerk = std::numeric_limits<double>::infinity();
  constexpr double kAccelerationDenominator = 18.0;
  constexpr double kAccelerationMultiplier = 2.0;
  constexpr double kJerkMultiplier = 4.5;
  constexpr double kSquareExponent = 2.0;
  constexpr double kCubeExponent = 3.0;
  constexpr double kCubicRootExponent = 1.0 / 3.0;
} // namespace

APConstraints::APConstraints()
    : APConstraints(kInfinityVelocity, kInfinityAcceleration, kInfinityJerk) {}

APConstraints::APConstraints(meters_per_second_t velocity,
                             meters_per_second_squared_t acceleration,
                             double jerk)
    : velocity_(velocity),
      acceleration_(acceleration),
      jerk_(jerk),
      x0_(std::pow(acceleration.value(), kCubeExponent) /
          (kAccelerationDenominator * jerk * jerk)),
      v0_(JerkConstrainedVelocity(x0_).value()) {}

APConstraints::APConstraints(meters_per_second_squared_t acceleration, double jerk)
    : APConstraints(kInfinityVelocity, acceleration, jerk) {}

APConstraints APConstraints::WithVelocity(meters_per_second_t newVelocity) const
{
  return APConstraints(newVelocity, acceleration_, jerk_);
}

APConstraints APConstraints::WithAcceleration(meters_per_second_squared_t newAcceleration) const
{
  return APConstraints(velocity_, newAcceleration, jerk_);
}

APConstraints APConstraints::WithJerk(double newJerk) const
{
  return APConstraints(velocity_, acceleration_, newJerk);
}

meters_per_second_t APConstraints::GetVelocity() const
{
  return velocity_;
}

meters_per_second_squared_t APConstraints::GetAcceleration() const
{
  return acceleration_;
}

double APConstraints::GetJerk() const
{
  return jerk_;
}

meters_per_second_t APConstraints::CalculateMaxVelocity(double dist) const
{
  if (dist > x0_)
  {
    return AccelerationConstrainedVelocity(dist);
  }
  return JerkConstrainedVelocity(dist);
}

meters_per_second_t APConstraints::AccelerationConstrainedVelocity(double dist) const
{
  return meters_per_second_t{
      std::sqrt(v0_ * v0_ + kAccelerationMultiplier * acceleration_.value() * (dist - x0_))};
}

meters_per_second_t APConstraints::JerkConstrainedVelocity(double dist) const
{
  return meters_per_second_t{
      std::pow((kJerkMultiplier * std::pow(dist, kSquareExponent)) * jerk_,
               kCubicRootExponent)};
}
