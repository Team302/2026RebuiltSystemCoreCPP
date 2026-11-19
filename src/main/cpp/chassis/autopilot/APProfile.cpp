#include "APProfile.h"

using wpi::units::angle::radian_t;
using wpi::units::length::meter_t;

namespace
{
  constexpr double kZero = 0.0;
} // namespace

APProfile::APProfile(const APConstraints &constraints)
    : m_constraints(constraints),
      m_errorXY(kZero),
      m_errorTheta(kZero),
      m_beelineRadius(kZero) {}

APProfile &APProfile::WithErrorXY(meter_t errorXY)
{
  m_errorXY = errorXY;
  return *this;
}

APProfile &APProfile::WithErrorTheta(radian_t errorTheta)
{
  m_errorTheta = errorTheta;
  return *this;
}

APProfile &APProfile::WithConstraints(const APConstraints &constraints)
{
  m_constraints = constraints;
  return *this;
}

APProfile &APProfile::WithBeelineRadius(meter_t beelineRadius)
{
  m_beelineRadius = beelineRadius;
  return *this;
}

meter_t APProfile::GetErrorXY() const
{
  return m_errorXY;
}

radian_t APProfile::GetErrorTheta() const
{
  return m_errorTheta;
}

const APConstraints &APProfile::GetConstraints() const
{
  return m_constraints;
}

APConstraints &APProfile::GetConstraints()
{
  return m_constraints;
}

meter_t APProfile::GetBeelineRadius() const
{
  return m_beelineRadius;
}
