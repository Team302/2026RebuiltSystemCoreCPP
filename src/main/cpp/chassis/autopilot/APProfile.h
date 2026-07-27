#pragma once

#include "wpi/units/angle.hpp"
#include "wpi/units/length.hpp"

#include "APConstraints.h"

class APProfile
{
public:
  explicit APProfile(const APConstraints &constraints);

  APProfile &WithErrorXY(wpi::units::length::meter_t errorXY);
  APProfile &WithErrorTheta(wpi::units::angle::radian_t errorTheta);
  APProfile &WithConstraints(const APConstraints &constraints);
  APProfile &WithBeelineRadius(wpi::units::length::meter_t beelineRadius);

  wpi::units::length::meter_t GetErrorXY() const;
  wpi::units::angle::radian_t GetErrorTheta() const;
  const APConstraints &GetConstraints() const;
  APConstraints &GetConstraints();
  wpi::units::length::meter_t GetBeelineRadius() const;

private:
  APConstraints constraints_;
  wpi::units::length::meter_t errorXY_;
  wpi::units::angle::radian_t errorTheta_;
  wpi::units::length::meter_t beelineRadius_;
};
