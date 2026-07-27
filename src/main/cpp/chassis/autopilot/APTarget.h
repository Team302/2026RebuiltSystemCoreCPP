#pragma once

#include <optional>

#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/math/geometry/Rotation2d.hpp"
#include "wpi/units/length.hpp"
#include "wpi/units/velocity.hpp"

using wpi::math::Rotation2d;

class APTarget
{
public:
  explicit APTarget(const wpi::math::Pose2d &pose);

  APTarget WithReference(const wpi::math::Pose2d &reference) const;
  APTarget WithEntryAngle(const wpi::math::Rotation2d &entryAngle) const;
  APTarget WithVelocity(wpi::units::velocity::meters_per_second_t velocity) const;
  APTarget WithRotationRadius(wpi::units::length::meter_t radius) const;

  const wpi::math::Pose2d &GetReference() const;
  const std::optional<wpi::math::Rotation2d> &GetEntryAngle() const;
  wpi::units::velocity::meters_per_second_t GetVelocity() const;
  const std::optional<wpi::units::length::meter_t> &GetRotationRadius() const;

  APTarget Clone() const;
  APTarget WithoutEntryAngle() const;

private:
  wpi::math::Pose2d reference_;
  std::optional<wpi::math::Rotation2d> entryAngle_;
  wpi::units::velocity::meters_per_second_t velocity_;
  std::optional<wpi::units::length::meter_t> rotationRadius_;
};
