#include "APTarget.h"

using wpi::math::Pose2d;
using wpi::math::Rotation2d;
using wpi::units::length::meter_t;
using wpi::units::velocity::meters_per_second_t;

namespace
{
  const meters_per_second_t kZero{0.0};
} // namespace

APTarget::APTarget(const Pose2d &pose)
    : reference_(pose), velocity_(kZero)
{
}

APTarget APTarget::WithReference(const Pose2d &reference) const
{
  APTarget target = Clone();
  target.reference_ = reference;
  return target;
}

APTarget APTarget::WithEntryAngle(const Rotation2d &entryAngle) const
{
  APTarget target = Clone();
  target.entryAngle_ = entryAngle;
  return target;
}

APTarget APTarget::WithVelocity(meters_per_second_t velocity) const
{
  APTarget target = Clone();
  target.velocity_ = velocity;
  return target;
}

APTarget APTarget::WithRotationRadius(meter_t radius) const
{
  APTarget target = Clone();
  target.rotationRadius_ = radius;
  return target;
}

const Pose2d &APTarget::GetReference() const
{
  return reference_;
}

const std::optional<Rotation2d> &APTarget::GetEntryAngle() const
{
  return entryAngle_;
}

meters_per_second_t APTarget::GetVelocity() const
{
  return velocity_;
}

const std::optional<meter_t> &APTarget::GetRotationRadius() const
{
  return rotationRadius_;
}

APTarget APTarget::Clone() const
{
  APTarget target(reference_);
  target.velocity_ = velocity_;
  target.entryAngle_ = entryAngle_;
  target.rotationRadius_ = rotationRadius_;
  return target;
}

APTarget APTarget::WithoutEntryAngle() const
{
  APTarget target(reference_);
  target.velocity_ = velocity_;
  target.rotationRadius_ = rotationRadius_;
  return target;
}
