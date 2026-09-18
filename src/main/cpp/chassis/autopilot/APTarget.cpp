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
    : m_reference(pose), m_velocity(kZero)
{
}

APTarget APTarget::WithReference(const Pose2d &reference) const
{
  APTarget target = Clone();
  target.m_reference = reference;
  return target;
}

APTarget APTarget::WithEntryAngle(const Rotation2d &entryAngle) const
{
  APTarget target = Clone();
  target.m_entryAngle = entryAngle;
  return target;
}

APTarget APTarget::WithVelocity(meters_per_second_t velocity) const
{
  APTarget target = Clone();
  target.m_velocity = velocity;
  return target;
}

APTarget APTarget::WithRotationRadius(meter_t radius) const
{
  APTarget target = Clone();
  target.m_rotationRadius = radius;
  return target;
}

const Pose2d &APTarget::GetReference() const
{
  return m_reference;
}

const std::optional<Rotation2d> &APTarget::GetEntryAngle() const
{
  return m_entryAngle;
}

meters_per_second_t APTarget::GetVelocity() const
{
  return m_velocity;
}

const std::optional<meter_t> &APTarget::GetRotationRadius() const
{
  return m_rotationRadius;
}

APTarget APTarget::Clone() const
{
  APTarget target(m_reference);
  target.m_velocity = m_velocity;
  target.m_entryAngle = m_entryAngle;
  target.m_rotationRadius = m_rotationRadius;
  return target;
}

APTarget APTarget::WithoutEntryAngle() const
{
  APTarget target(m_reference);
  target.m_velocity = m_velocity;
  target.m_rotationRadius = m_rotationRadius;
  return target;
}
