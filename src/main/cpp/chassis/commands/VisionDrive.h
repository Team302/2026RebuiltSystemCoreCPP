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
#include <optional>

#include "chassis/autopilot/APConstraints.h"
#include "chassis/autopilot/APProfile.h"
#include "chassis/autopilot/APTarget.h"
#include "chassis/autopilot/Autopilot.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "teleopcontrol/TeleopControl.h"
#include "wpi/commands2/Command.hpp"
#include "wpi/commands2/CommandHelper.hpp"
#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/units/angular_velocity.hpp"
#include "wpi/units/velocity.hpp"

//====================================================================================================================================================
/// @class VisionDrive
/// @brief Base class for vision-guided driving with autopilot setpoint generation and manual fallback.
/// @details Derived classes provide `APTarget` data from camera observations via `GetAutopilotTarget()`.
///          When a target is available, this base class computes motion with `Autopilot::Calculate()` and
///          applies a field-centric facing-angle request. When no target is available, it falls back to
///          driver joystick field-centric control.
//====================================================================================================================================================
class VisionDrive : public wpi::cmd::Command
{
public:
    /// @brief Constructs a vision-assisted drive command base.
    /// @param chassis Swerve drivetrain used to apply drive requests.
    VisionDrive(subsystems::CommandSwerveDrivetrain *chassis);

    /// @brief Initializes autopilot constraints/profile for the active derived command.
    void Initialize() override;

    /// @brief Runs either manual fallback drive or autopilot tracking when a target is available.
    void Execute() override;

    /// @brief VisionDrive is intended to run until interrupted.
    /// @return Always false.
    bool IsFinished() override;

    /// @brief Stops the drivetrain when the command ends.
    /// @param interrupted True if canceled by another command.
    void End(bool interrupted) override;

    /// @brief Gets max translational speed used by manual fallback and autopilot constraints.
    /// @return Max speed in meters per second.
    virtual wpi::units::velocity::meters_per_second_t GetMaxVelocity() const { return kMaxVelocityDefault; }

protected:
    /// @brief Returns the current autopilot target for this cycle.
    /// @return A target when vision has a valid solution; std::nullopt to use manual fallback drive.
    virtual std::optional<APTarget> GetAutopilotTarget() = 0;

    /// @brief XY completion tolerance for autopilot.
    /// @return Positional tolerance in meters.
    virtual wpi::units::length::meter_t GetAutopilotErrorXY() const { return 0.02_m; }

    /// @brief Heading completion tolerance for autopilot.
    /// @return Angular tolerance in radians.
    virtual wpi::units::angle::radian_t GetAutopilotErrorTheta() const { return 0.5_deg; }

    /// @brief Radius where beeline logic transitions to entry-angle shaping.
    /// @return Beeline radius in meters.
    virtual wpi::units::length::meter_t GetAutopilotBeelineRadius() const { return 0.08_m; }

    subsystems::CommandSwerveDrivetrain *m_chassis;

private:
    TeleopControl *m_controller = TeleopControl::GetInstance();
    wpi::units::velocity::meters_per_second_t m_maxSpeed = 4_mps;
    wpi::units::angular_velocity::degrees_per_second_t m_maxAngularRate = 360_deg_per_s;

    /// @brief Maximum translational velocity for the robot
    wpi::units::velocity::meters_per_second_t kMaxVelocity;
    static constexpr wpi::units::velocity::meters_per_second_t kMaxVelocityDefault = 4_mps;

    wpi::units::acceleration::meters_per_second_squared_t kMaxAcceleration;
    static constexpr wpi::units::acceleration::meters_per_second_squared_t kMaxAccelerationDefault = 3_mps_sq;

    double m_maxJerk = 2.0;

    swerve::requests::FieldCentric m_fieldDriveRequest = swerve::requests::FieldCentric{}
                                                             .WithDeadband(m_maxSpeed * 0.1)                                  // TODO: Investigate this deadband vs controller deadband
                                                             .WithRotationalDeadband(m_maxAngularRate * 0.1)                  // TODO: Investigate this deadband vs controller deadband
                                                             .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage) // Use open-loop voltage for drive
                                                             .WithDesaturateWheelVelocities(true);                            // Don't desaturate wheel velocities, since we'll be capping the max velocity of the robot

    swerve::requests::FieldCentricFacingAngle m_driveRequest;

    APConstraints m_autopilotConstraints{kMaxVelocityDefault, kMaxAccelerationDefault, m_maxJerk};
    APProfile m_autopilotProfile{m_autopilotConstraints};
    Autopilot m_autopilot{m_autopilotProfile};
};