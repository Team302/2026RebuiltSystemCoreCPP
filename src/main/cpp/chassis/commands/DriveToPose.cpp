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
/// @file DriveToPose.cpp
/// @brief Implementation of DriveToPose command for autonomous pose-based navigation
/// @details This command uses the Team 302 C++ Autopilot classes to generate field-relative
///          translational and heading setpoints toward one or two target poses. Derived classes
///          provide the target pose(s), while this base class handles target staging, control
///          application, completion checks, and state publication.
///
///          The command is designed to be extended by derived classes that provide specific target poses
///          for different field elements or game-specific locations.
//====================================================================================================================================================

#include "chassis/commands/DriveToPose.h"
#include "chassis/autopilot/APConstraints.h"
#include "chassis/autopilot/APProfile.h"
#include "state/RobotState.h"
#include "utils/AngleUtils.h"
#include "utils/PoseUtils.h"
#include "utils/logging/debug/Logger.h"
#include "wpi/math/geometry/Rotation2d.hpp"
#include "wpi/math/geometry/Translation2d.hpp"

using wpi::units::acceleration::meters_per_second_squared_t;
using wpi::units::angle::degree_t;
using wpi::units::angle::radian_t;
using wpi::units::length::meter_t;
using wpi::units::velocity::meters_per_second_t;

static constexpr meter_t AUTOPILOT_XY_ERROR{0.02};
static constexpr degree_t AUTOPILOT_THETA_ERROR{0.5};
static constexpr meter_t AUTOPILOT_BEELINE_RADIUS{0.08};
static constexpr double ROTATION_KP{4.5};
static constexpr double ROTATION_KI{0.0};
static constexpr double ROTATION_KD{0.0};

//------------------------------------------------------------------
/// @brief      Constructor for DriveToPose command
/// @param[in]  chassis - Pointer to the swerve drive subsystem used for navigation
/// @details    Initializes default autopilot profile parameters (XY/theta tolerances,
///             beeline radius), stores initial pose for movement detection, and captures
///             chassis subsystem ownership.
///
///             **Initial State:**
///             - Captures current robot pose as the previous pose reference
///             - Calculates feedforward range (max radius - min radius) for velocity scaling
///             - Registers chassis as a required subsystem
///
///             **Control Strategy:**
///             Motion setpoints are generated each cycle by `Autopilot::Calculate()`.
///
/// @note       The command requires exclusive access to the chassis subsystem
/// @see        Initialize() for per-run setup when command is scheduled
//------------------------------------------------------------------
DriveToPose::DriveToPose(subsystems::CommandSwerveDrivetrain *chassis) : m_chassis(chassis)
{
    AddRequirements(m_chassis);

    m_autopilotProfile.WithErrorXY(AUTOPILOT_XY_ERROR)
        .WithErrorTheta(AUTOPILOT_THETA_ERROR)
        .WithConstraints(m_autopilotConstraints)
        .WithBeelineRadius(AUTOPILOT_BEELINE_RADIUS);
}

//------------------------------------------------------------------
/// @brief      Initializes the command when it starts execution
/// @details    Called once by the command scheduler when the command is first scheduled.
///             Performs the following initialization steps:
///
///             **Target Acquisition:**
///             - Calls GetDriveToPoses() to determine the target pose(s) (overridable by derived classes)
///             - Sets the end pose and optional mid pose as navigation goals
///
///             **State Preparation:**
///             - Resets the "same pose" tracker to detect when robot stops moving
///             - Updates the target pose for the control system
///
///             **Robot State Publishing:**
///             - Publishes DriveToFieldElement_Bool = true (navigation active)
///             - Publishes DriveToFinished_Bool = false (not yet complete)
///
///             These state changes notify other subsystems that autonomous navigation is in progress,
///             which may trigger coordinated behaviors or safety interlocks.
///
/// @note       Derived classes can override GetDriveToPoses() to provide custom target calculations
/// @see        GetDriveToPoses() for target pose determination
/// @see        SetTargetPose() for controller configuration
//------------------------------------------------------------------
void DriveToPose::Initialize()
{
    // Re-read velocity/acceleration limits here (not in constructor) because virtual dispatch
    // does not work during base-class construction — derived overrides (e.g. DriveOverBump)
    kMaxVelocity = GetMaxVelocity();
    kMaxAcceleration = GetMaxAcceleration();
    m_autopilotConstraints = m_autopilotConstraints.WithAcceleration(kMaxAcceleration)
                                 .WithVelocity(kMaxVelocity)
                                 .WithJerkMetersPerSecondCubed(m_maxJerk);
    m_autopilotProfile.WithConstraints(m_autopilotConstraints);
    m_autopilot = Autopilot(m_autopilotProfile);

    // Get the target pose (may be overridden by derived classes)
    auto poses = GetDriveToPoses();
    m_midPointSpeed = poses.midPointSpeed;
    m_endPointSpeed = poses.endPointSpeed;
    m_endPose = poses.endPose;
    m_hasMidPose = poses.hasMidPose;

    if (poses.hasMidPose)
    {
        m_midPose = poses.midPose;
        m_beforeMidPose = ShouldSkipMidPoint() ? false : true; // Skip mid pose if angle to target is small
    }
    else
    {
        m_beforeMidPose = false;
    }

    // Reset movement detection to start fresh
    m_chassis->ResetSamePose();

    // Reset completion flag so each run starts clean
    m_isFinished = false;

    // Set current target speed based on which pose we're navigating to
    m_currentTargetSpeed = m_beforeMidPose ? m_midPointSpeed : m_endPointSpeed;

    // Configure autopilot target for the active stage
    SetTargetPose(m_beforeMidPose ? m_midPose : m_endPose);

    // Notify robot state that navigation has started
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFieldElement_Bool, true);
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFinished_Bool, false);
}

//------------------------------------------------------------------
/// @brief      Updates the active target pose and autopilot target
/// @param[in]  endPose - The new target pose to navigate to
/// @details    This method allows the target pose to be changed during command execution,
///             useful for multi-stage navigation or dynamic target updates.
///
///             **Target Update:**
///             - Stores the new pose in `m_targetPose`
///             - Rebuilds `m_autopilotTarget` with updated reference and stage speed
///
///             This is used internally during Initialize() and can be called by derived classes
///             to implement multi-waypoint navigation (e.g., DriveOverBump's two-stage approach).
///
/// @note       Calling this during execution will cause the robot to smoothly redirect to the new target
/// @see        DriveOverBump::IsFinished() for an example of dynamic target switching
//------------------------------------------------------------------
void DriveToPose::SetTargetPose(const wpi::math::Pose2d &pose)
{
    m_targetPose = pose;
    if (m_chassis != nullptr)
    {
        // Update current pose and set new controller goals
        m_currentPose = m_chassis->GetPose();
        m_autopilotTarget = m_autopilotTarget.WithReference(m_targetPose)
                                .WithVelocity(m_currentTargetSpeed);
    }
}

//------------------------------------------------------------------
/// @brief      Executes the command periodically during autonomous navigation
/// @details    Called repeatedly by the command scheduler (typically every 20ms) while the command is active.
///             Uses `Autopilot::Calculate()` to obtain field-relative translational setpoints and
///             heading target, then applies those values to the drivetrain request.
///
///             **Control Flow:**
///             1. Update current robot pose from odometry/vision fusion
///             2. Build robot-relative speed input from chassis state
///             3. Run autopilot calculate
///             4. Apply output to field-centric-facing-angle request
///             5. Log velocity/error values for tuning
///
/// @note       All movements are field-centric relative to blue alliance perspective
/// @see        IsFinished() for completion detection
//------------------------------------------------------------------
void DriveToPose::Execute()
{
    wpi::math::ChassisVelocities speeds{};
    if (m_chassis != nullptr)
    {
        // Update current position from odometry
        m_currentPose = m_chassis->GetPose();
        auto robotRelativeSpeeds = m_chassis->GetState().Velocity;
        auto output = m_autopilot.Calculate(m_currentPose, robotRelativeSpeeds, m_autopilotTarget);
        speeds.vx = output.vx;
        speeds.vy = output.vy;
        m_distanceError = m_currentPose.Translation().Distance(m_targetPose.Translation());

        // Send control request to drivetrain with heading control
        m_chassis->SetControl(
            m_driveRequest.WithVelocityX(speeds.vx)
                .WithVelocityY(speeds.vy)
                .WithTargetDirection(output.targetAngle)
                .WithHeadingPID(ROTATION_KP, ROTATION_KI, ROTATION_KD)
                .WithForwardPerspective(ctre::phoenix6::swerve::requests::ForwardPerspectiveValue::BlueAlliance));
    }

    // Log current error for debugging and tuning (reuse m_distanceError computed in CalculateFeedForward)
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToPose", "Vx", wpi::units::math::abs(speeds.vx).value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToPose", "Vy", wpi::units::math::abs(speeds.vy).value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToPose", "Error", wpi::units::length::inch_t(m_distanceError).value());
}

//------------------------------------------------------------------
/// @brief      Determines if the command has completed navigation to the target pose
/// @return     bool - true if navigation is complete or robot is stuck, false to continue execution
/// @details    Uses autopilot target completion plus stuck detection:
///
///             **Condition 1: Target is at the origin**
///             - Checks if the end pose is at the origin (0,0) with a small tolerance
///             - Returns true if end pose is at origin, indicating an error in target calculation (physically the robot
///               center cannot be at the origin of the field).
///
///             **Condition 2: Target Reached**
///             - Uses `Autopilot::AtTarget()` with profile tolerances
///             - Indicates successful navigation completion
///
///             **Condition 3: Robot Stopped Moving**
///             - Uses chassis IsSamePose() to detect if robot hasn't moved between cycles
///             - Returns true if robot is stuck or unable to make progress
///             - Prevents command from running indefinitely if blocked
///
///             **Debug Logging:**
///             - Logs "Is Done" status (target reached)
///             - Logs "Is SamePose" status (movement detection)
///             - Useful for tuning autopilot profile and diagnosing navigation issues
///
///             The stuck detection provides graceful failure handling - if the robot can't reach
///             the target (blocked by game pieces, defense, field obstacles), the command will
///             eventually timeout rather than running indefinitely.
///
/// @note       Legacy distance-threshold field is retained for compatibility, but primary completion
///             uses autopilot profile tolerances.
//------------------------------------------------------------------
bool DriveToPose::IsFinished()
{
    // Safety check: If end pose wasn't calculated properly, stop immediately
    if (PoseUtils::IsPoseAtOrigin(m_targetPose, wpi::units::length::centimeter_t{1.0}))
    {
        m_isFinished = true;
        return true;
    }

    // Check if we've reached the target pose within autopilot profile tolerances
    bool isDone = m_autopilot.AtTarget(m_currentPose, m_autopilotTarget);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToPose", "Is Done", isDone);

    // Handle transition from mid pose to end pose
    if (m_hasMidPose && m_beforeMidPose)
    {
        // Transition if mid pose reached or within tolerance to end pose
        if (isDone ||
            (wpi::units::math::abs(m_endPose.X() - m_currentPose.X()) < m_xtoleranceForTransitionToEndPoint &&
             wpi::units::math::abs(m_endPose.Y() - m_currentPose.Y()) < m_yToleranceForTransitionToEndPoint) ||
            ShouldSkipMidPoint())
        {
            m_currentTargetSpeed = m_endPointSpeed;
            SetTargetPose(m_endPose);
            m_beforeMidPose = false;
            m_isFinished = false;
            return false; // Continue execution to reach end pose
        }
    }

    // Check if robot has stopped moving (stuck or blocked)
    bool isSamePose = m_chassis->IsSamePose();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToPose", "Is SamePose", isSamePose);

    return (m_isFinished = isDone || isSamePose);
}

//------------------------------------------------------------------
/// @brief      Cleans up and stops the robot when the command ends
/// @param[in]  interrupted - true if the command was interrupted by another command,
///                          false if it finished normally via IsFinished()
/// @details    Called once by the command scheduler when the command terminates.
///             Performs cleanup operations:
///
///             **Robot Control:**
///             - Applies SwerveDriveBrake to immediately stop all wheel motion
///             - Ensures robot doesn't coast after command ends
///             - Important for safety and precise positioning
///
///             **State Publishing:**
///             - Updates DriveToFinished_Bool with completion status from IsFinished()
///             - Updates DriveToFieldElement_Bool to false (navigation no longer active)
///             - Notifies other subsystems that autonomous navigation has ended
///
///             The interrupted parameter indicates how the command ended:
///             - false: Command completed normally (reached target or detected stuck)
///             - true: Command was preempted by another command or manually cancelled
///
///             In both cases, the robot is stopped and state is updated to reflect
///             that navigation is no longer active.
///
/// @note       This is called automatically by the command scheduler, not manually
/// @see        Initialize() for the corresponding startup method
//------------------------------------------------------------------
void DriveToPose::End(bool interrupted)
{
    // Immediately stop the robot
    if (m_chassis != nullptr)
    {
        m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
    }

    // Publish completion status to robot state (use cached result to avoid redundant recomputation)
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFinished_Bool, m_isFinished);
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFieldElement_Bool, false);
}

/**
 * @brief Determines whether the midpoint target should be skipped based on the robot's alignment with the end pose.
 *
 * This method calculates the angle between the robot's current position and the final target position.
 * If the robot is already aligned within the specified angle tolerance, the midpoint can be skipped
 * for a more direct path to the end pose.
 *
 * @return true if the angle to the end pose is within the angle tolerance and the midpoint should be skipped
 * @return false if the robot should navigate through the midpoint before reaching the end pose
 */
bool DriveToPose::ShouldSkipMidPoint() const
{
    auto currentPose = m_chassis->GetPose();
    auto rotationToEnd = (currentPose.Translation() - m_endPose.Translation()).Angle();
    if (!rotationToEnd.has_value())
    {
        return true;
    }
    auto angleToEnd = wpi::units::math::abs(AngleUtils::GetEquivAngle(rotationToEnd->Degrees()));
    angleToEnd = std::min(angleToEnd, 180.0_deg - angleToEnd);
    return angleToEnd < m_angleTolerance;
}
