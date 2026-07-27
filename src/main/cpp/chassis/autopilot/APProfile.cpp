#include "APProfile.h"

using wpi::units::angle::radian_t;
using wpi::units::length::meter_t;

namespace
{
  constexpr double kZero = 0.0;
} // namespace

APProfile::APProfile(const APConstraints &constraints)
    : constraints_(constraints),
      errorXY_(kZero),
      errorTheta_(kZero),
      beelineRadius_(kZero) {}

APProfile &APProfile::WithErrorXY(meter_t errorXY)
{
  errorXY_ = errorXY;
  return *this;
}

APProfile &APProfile::WithErrorTheta(radian_t errorTheta)
{
  errorTheta_ = errorTheta;
  return *this;
}

APProfile &APProfile::WithConstraints(const APConstraints &constraints)
{
  constraints_ = constraints;
  return *this;
}

APProfile &APProfile::WithBeelineRadius(meter_t beelineRadius)
{
  beelineRadius_ = beelineRadius;
  return *this;
}

meter_t APProfile::GetErrorXY() const
{
  return errorXY_;
}

radian_t APProfile::GetErrorTheta() const
{
  return errorTheta_;
}

const APConstraints &APProfile::GetConstraints() const
{
  return constraints_;
}

APConstraints &APProfile::GetConstraints()
{
  return constraints_;
}

meter_t APProfile::GetBeelineRadius() const
{
  return beelineRadius_;
}
