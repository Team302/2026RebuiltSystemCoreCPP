#include "Autopilot.h"

#include <algorithm>
#include <cmath>

#include "wpi/math/kinematics/ChassisVelocities.hpp"

using wpi::math::Rotation2d;

namespace
{
  constexpr double kZero = 0.0;
  constexpr double kOne = 1.0;
  constexpr double kHalf = 0.5;
  constexpr double kLoopPeriodSeconds = 0.020;
} // namespace

Autopilot::Autopilot(const APProfile &profile) : profile_(profile), dt_(kLoopPeriodSeconds) {}

Autopilot::APResult Autopilot::Calculate(const wpi::math::Pose2d &current,
                                         const wpi::math::ChassisVelocities &robotRelativeSpeeds,
                                         const APTarget &target) const
{
  wpi::math::Translation2d offset = ToTargetCoordinateFrame(
      target.GetReference().Translation() - current.Translation(), target);

  if (offset == wpi::math::Translation2d{})
  {
    return APResult{wpi::units::meters_per_second_t{kZero},
                    wpi::units::meters_per_second_t{kZero},
                    target.GetReference().Rotation()};
  }

  wpi::math::Translation2d fieldRelativeSpeeds{
      wpi::units::length::meter_t{robotRelativeSpeeds.vx.value()},
      wpi::units::length::meter_t{robotRelativeSpeeds.vy.value()}};
  fieldRelativeSpeeds = fieldRelativeSpeeds.RotateBy(current.Rotation());

  wpi::math::Translation2d initial = ToTargetCoordinateFrame(fieldRelativeSpeeds, target);
  double disp = offset.Norm().value();

  if (!target.GetEntryAngle().has_value() ||
      disp < profile_.GetBeelineRadius().value())
  {
    wpi::math::Translation2d towardsTarget = offset / disp;
    wpi::math::Translation2d goal = towardsTarget *
                                    (profile_.GetConstraints().CalculateMaxVelocity(disp) + target.GetVelocity()).value();
    wpi::math::Translation2d out = Correct(initial, goal);
    wpi::math::Translation2d velo = ToGlobalCoordinateFrame(out, target);
    wpi::math::Rotation2d rot = GetRotationTarget(current.Rotation(), target, disp);
    return APResult{wpi::units::meters_per_second_t{velo.X().value()},
                    wpi::units::meters_per_second_t{velo.Y().value()}, rot};
  }

  wpi::math::Translation2d goal = CalculateSwirlyVelocity(offset, target);
  wpi::math::Translation2d out = Correct(initial, goal);
  wpi::math::Translation2d velo = ToGlobalCoordinateFrame(out, target);
  wpi::math::Rotation2d rot = GetRotationTarget(current.Rotation(), target, disp);
  return APResult{wpi::units::meters_per_second_t{velo.X().value()},
                  wpi::units::meters_per_second_t{velo.Y().value()}, rot};
}

wpi::math::Translation2d Autopilot::ToTargetCoordinateFrame(
    const wpi::math::Translation2d &coords, const APTarget &target) const
{
  wpi::math::Rotation2d entryAngle =
      target.GetEntryAngle().has_value() ? target.GetEntryAngle().value() : wpi::math::Rotation2d{};
  return coords.RotateBy(-entryAngle);
}

wpi::math::Translation2d Autopilot::ToGlobalCoordinateFrame(
    const wpi::math::Translation2d &coords, const APTarget &target) const
{
  wpi::math::Rotation2d entryAngle =
      target.GetEntryAngle().has_value() ? target.GetEntryAngle().value() : wpi::math::Rotation2d{};
  return coords.RotateBy(entryAngle);
}

wpi::math::Translation2d Autopilot::Correct(const wpi::math::Translation2d &initial,
                                            const wpi::math::Translation2d &goal) const
{
  wpi::math::Rotation2d angleOffset{};
  if (goal != wpi::math::Translation2d{})
  {
    angleOffset = wpi::math::Rotation2d(goal.X().value(), goal.Y().value());
  }

  wpi::math::Translation2d adjustedGoal = goal.RotateBy(-angleOffset);
  wpi::math::Translation2d adjustedInitial = initial.RotateBy(-angleOffset);
  double initialI = adjustedInitial.X().value();
  double goalI = adjustedGoal.X().value();

  if (goalI > profile_.GetConstraints().GetVelocity().value())
  {
    goalI = profile_.GetConstraints().GetVelocity().value();
  }

  double adjustedI = std::min(
      goalI, Push(initialI, goalI, profile_.GetConstraints().GetAcceleration().value()));
  return wpi::math::Translation2d(wpi::units::length::meter_t{adjustedI}, wpi::units::length::meter_t{kZero})
      .RotateBy(angleOffset);
}

double Autopilot::Push(double start, double end, double accel) const
{
  double maxChange = accel * dt_;
  if (std::abs(start - end) < maxChange)
  {
    return end;
  }
  if (start > end)
  {
    return start - maxChange;
  }
  return start + maxChange;
}

wpi::math::Translation2d Autopilot::CalculateSwirlyVelocity(
    const wpi::math::Translation2d &offset, const APTarget &target) const
{
  double disp = offset.Norm().value();
  wpi::math::Rotation2d theta(offset.X().value(), offset.Y().value());
  double rads = theta.Radians().value();
  double dist = CalculateSwirlyLength(rads, disp);

  double vx = theta.Cos() - rads * theta.Sin();
  double vy = rads * theta.Cos() + theta.Sin();

  wpi::math::Translation2d velocityVector(wpi::units::length::meter_t{vx}, wpi::units::length::meter_t{vy});
  velocityVector = velocityVector / std::hypot(vx, vy);
  return velocityVector *
         (profile_.GetConstraints().CalculateMaxVelocity(dist) + target.GetVelocity()).value();
}

double Autopilot::CalculateSwirlyLength(double theta, double radius) const
{
  if (theta == kZero)
  {
    return radius;
  }

  theta = std::abs(theta);
  double hypot = std::hypot(theta, kOne);
  double u1 = radius * hypot;
  double u2 = radius * std::log(theta + hypot) / theta;
  return kHalf * (u1 + u2);
}

wpi::math::Rotation2d Autopilot::GetRotationTarget(const wpi::math::Rotation2d &current,
                                                   const APTarget &target,
                                                   double dist) const
{
  if (!target.GetRotationRadius().has_value())
  {
    return target.GetReference().Rotation();
  }

  double radius = target.GetRotationRadius().value().value();
  if (radius > dist)
  {
    return target.GetReference().Rotation();
  }
  return current;
}

bool Autopilot::AtTarget(const wpi::math::Pose2d &current, const APTarget &target) const
{
  wpi::math::Pose2d goal = target.GetReference();
  bool okXY = std::hypot((current.X() - goal.X()).value(),
                         (current.Y() - goal.Y()).value()) <=
              profile_.GetErrorXY().value();
  bool okTheta =
      std::abs((current.Rotation() - goal.Rotation()).Radians().value()) <=
      profile_.GetErrorTheta().value();
  return okXY && okTheta;
}
