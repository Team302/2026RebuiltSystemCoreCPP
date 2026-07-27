#pragma once

#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/math/geometry/Rotation2d.hpp"
#include "wpi/math/geometry/Translation2d.hpp"
#include "wpi/math/kinematics/ChassisVelocities.hpp"
#include "wpi/units/velocity.hpp"

#include "APProfile.h"
#include "APTarget.h"

class Autopilot
{
public:
  struct APResult
  {
    wpi::units::meters_per_second_t vx;
    wpi::units::meters_per_second_t vy;
    wpi::math::Rotation2d targetAngle;
  };

  explicit Autopilot(const APProfile &profile);

  APResult Calculate(const wpi::math::Pose2d &current,
                     const wpi::math::ChassisVelocities &robotRelativeSpeeds,
                     const APTarget &target) const;

  bool AtTarget(const wpi::math::Pose2d &current, const APTarget &target) const;

private:
  wpi::math::Translation2d ToTargetCoordinateFrame(const wpi::math::Translation2d &coords,
                                                   const APTarget &target) const;

  wpi::math::Translation2d ToGlobalCoordinateFrame(const wpi::math::Translation2d &coords,
                                                   const APTarget &target) const;

  wpi::math::Translation2d Correct(const wpi::math::Translation2d &initial,
                                   const wpi::math::Translation2d &goal) const;

  double Push(double start, double end, double accel) const;

  wpi::math::Translation2d CalculateSwirlyVelocity(const wpi::math::Translation2d &offset,
                                                   const APTarget &target) const;

  double CalculateSwirlyLength(double theta, double radius) const;

  wpi::math::Rotation2d GetRotationTarget(const wpi::math::Rotation2d &current,
                                          const APTarget &target,
                                          double dist) const;

  APProfile profile_;
  double dt_;
};
