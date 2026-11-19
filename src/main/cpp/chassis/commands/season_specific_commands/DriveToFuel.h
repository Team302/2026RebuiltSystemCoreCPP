//====================================================================================================================================================
// Copyright 2026 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================
#pragma once

#include <memory>

#include "chassis/commands/VisionDrive.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "vision/DragonVision.h"
#include "wpi/commands2/CommandHelper.hpp"

class DriveToFuel : public wpi::cmd::CommandHelper<VisionDrive, DriveToFuel>
{
public:
    /// @brief Constructs vision-based drive-to-fuel alignment command.
    /// @param chassis Swerve drivetrain used for autopilot control.
    DriveToFuel(subsystems::CommandSwerveDrivetrain *chassis);

protected:
    /// @brief Builds an autopilot target from the closest valid fuel target observation.
    /// @return Target pose when vision has a valid object; std::nullopt otherwise.
    std::optional<APTarget> GetAutopilotTarget() override;

    /// @brief Uses tighter XY tolerance for more repeatable approach distance.
    wpi::units::length::meter_t GetAutopilotErrorXY() const override { return 0.03_m; }

    /// @brief Uses tight heading tolerance to improve fuel alignment.
    wpi::units::angle::radian_t GetAutopilotErrorTheta() const override { return 0.25_deg; }

    /// @brief Smaller beeline radius for smoother close-in corrections.
    wpi::units::length::meter_t GetAutopilotBeelineRadius() const override { return 0.06_m; }

private:
    DragonVision *m_vision = nullptr;

    static constexpr wpi::units::length::meter_t m_XdistLimit = 5_m;
    static constexpr wpi::units::length::meter_t m_IntakeXOffset = 0.5_m;

    std::vector<std::unique_ptr<DragonVisionStruct>> m_visionCache;
};