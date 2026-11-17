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

//====================================================================================================================================================
/// @file VisionDrive.cpp
/// @brief Base implementation for vision-guided autopilot driving with manual fallback.
/// @details Configures command-level autopilot constraints/tolerances, computes autopilot
///          outputs when derived classes provide targets, and otherwise applies driver field control.
//====================================================================================================================================================

#include "chassis/commands/VisionDrive.h"
#include "utils/logging/debug/Logger.h"

//------------------------------------------------------------------
/// @brief      Constructs the VisionDrive base command.
/// @param[in]  chassis Swerve drivetrain subsystem to control.
//------------------------------------------------------------------
VisionDrive::VisionDrive(subsystems::CommandSwerveDrivetrain *chassis) : m_chassis(chassis)
{
    AddRequirements(m_chassis);
}

//------------------------------------------------------------------
/// @brief      Initializes command-level autopilot profile and constraints.
/// @details    Rebuilds constraints from command speed/accel limits, applies virtual
///             tolerance hooks from derived commands, and logs effective tuning values.
//------------------------------------------------------------------
void VisionDrive::Initialize()
{
    m_maxSpeed = GetMaxVelocity();
    kMaxVelocity = m_maxSpeed;
    kMaxAcceleration = kMaxAccelerationDefault;

    m_autopilotConstraints = m_autopilotConstraints.WithAcceleration(kMaxAcceleration)
                                 .WithVelocity(kMaxVelocity)
                                 .WithJerkMetersPerSecondCubed(m_maxJerk);
    m_autopilotProfile.WithErrorXY(GetAutopilotErrorXY())
        .WithErrorTheta(GetAutopilotErrorTheta())
        .WithBeelineRadius(GetAutopilotBeelineRadius())
        .WithConstraints(m_autopilotConstraints);
    m_autopilot = Autopilot(m_autopilotProfile);

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "AP ErrorXY (m)", m_autopilotProfile.GetErrorXY().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "AP ErrorTheta (deg)", wpi::units::angle::degree_t(m_autopilotProfile.GetErrorTheta()).value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "AP BeelineRadius (m)", m_autopilotProfile.GetBeelineRadius().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "AP MaxVelocity (mps)", m_autopilotConstraints.GetVelocity().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "AP MaxAccel (mps2)", m_autopilotConstraints.GetAcceleration().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "AP MaxJerk (mps3)", m_autopilotConstraints.GetJerkMetersPerSecondCubed());
}

//------------------------------------------------------------------
/// @brief      Executes one control cycle.
/// @details    If a derived command provides an autopilot target, computes field-relative
///             setpoints with autopilot and applies them. Otherwise, falls back to manual
///             field-centric joystick driving.
//------------------------------------------------------------------
void VisionDrive::Execute()
{
    auto autopilotTarget = GetAutopilotTarget();
    if (!autopilotTarget.has_value())
    {
        double forward = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD);
        double strafe = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE);
        double rotate = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE);

        m_chassis->SetControl(
            m_fieldDriveRequest.WithVelocityX(forward * m_maxSpeed)
                .WithVelocityY(strafe * m_maxSpeed)
                .WithRotationalRate(rotate * m_maxAngularRate));
    }
    else
    {
        auto currentPose = m_chassis->GetPose();
        auto robotRelativeSpeeds = m_chassis->GetState().Velocity;
        auto output = m_autopilot.Calculate(currentPose, robotRelativeSpeeds, autopilotTarget.value());

        m_chassis->SetControl(
            m_driveRequest.WithVelocityX(output.vx)
                .WithVelocityY(output.vy)
                .WithTargetDirection(output.targetAngle)
                .WithForwardPerspective(ctre::phoenix6::swerve::requests::ForwardPerspectiveValue::BlueAlliance));
    }
}

//------------------------------------------------------------------
/// @brief      VisionDrive commands run until interrupted.
/// @return     Always false.
//------------------------------------------------------------------
bool VisionDrive::IsFinished()
{
    // A default drive command should never finish on its own.
    // It runs until it is interrupted by another command.
    return false;
}

//------------------------------------------------------------------
/// @brief      Stops drivetrain motion when command ends.
/// @param[in]  interrupted True if interrupted by another command.
//------------------------------------------------------------------
void VisionDrive::End(bool interrupted)
{
    m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
}