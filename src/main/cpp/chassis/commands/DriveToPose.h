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

#include "chassis/autopilot/APConstraints.h"
#include "chassis/autopilot/APProfile.h"
#include "chassis/autopilot/APTarget.h"
#include "chassis/autopilot/Autopilot.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "wpi/commands2/Command.hpp"
#include "wpi/commands2/CommandHelper.hpp"
#include "wpi/math/geometry/Pose2d.hpp"

struct DriveToPoses
{
    wpi::math::Pose2d endPose{};
    bool hasMidPose = false;
    wpi::math::Pose2d midPose{};
    wpi::units::velocity::meters_per_second_t midPointSpeed{0.0_mps};
    wpi::units::velocity::meters_per_second_t endPointSpeed{0.0_mps};
};

//====================================================================================================================================================
/// @class DriveToPose
/// @brief Base command class for autonomous navigation to field poses using Autopilot setpoint generation
///
/// This command implements a sophisticated control strategy for driving a swerve drivetrain to specific
/// field poses with high accuracy and smooth motion profiles. It serves as a base class that can be
/// extended for navigation to specific field elements or game pieces.
///
/// **Control Architecture:**
/// The command delegates translational and heading setpoint generation to `Autopilot`:
/// - Builds `APConstraints` and `APProfile` from command limits and tolerances
/// - Updates an `APTarget` for the active stage (midpoint or endpoint)
/// - Calls `Autopilot::Calculate()` each cycle to get field-relative X/Y velocity and heading
/// - Uses `Autopilot::AtTarget()` for completion checks
///
/// **Motion Profile:**
/// - Maximum velocity/acceleration come from `GetMaxVelocity()` / `GetMaxAcceleration()`
/// - Autopilot profile defaults are configured in constructor and refreshed in `Initialize()`
/// - Completion tolerances are based on autopilot XY/theta profile tolerances
///
/// **Extensibility:**
/// Derived classes override GetDriveToPoses() to provide specific target calculations:
/// - DriveToDepot: Navigate to depot zones
/// - DriveOverBump: Multi-stage bump crossing
/// - DriveToGamePiece: Vision-based game piece targeting
///
/// **State Management:**
/// The command publishes state changes to RobotState for coordination with other subsystems
/// and provides logging for debugging and performance analysis.
///
/// @see DriveOverBump Example of multi-waypoint navigation using SetTargetPose()
/// @see CommandSwerveDrivetrain The swerve drive subsystem controlled by this command
//====================================================================================================================================================
class DriveToPose : public wpi::cmd::CommandHelper<wpi::cmd::Command, DriveToPose>
{
public:
    //------------------------------------------------------------------
    /// @brief      Constructs a DriveToPose command for autonomous navigation
    /// @param[in]  chassis - Pointer to the swerve drive subsystem that will execute the motion
    /// @details    Captures chassis dependency and initializes default autopilot profile values
    ///             (error XY, error theta, and beeline radius). The command requires
    ///             exclusive access to the chassis subsystem during execution.
    ///
    /// @note       Derived classes should call this constructor via initializer list
    /// @see        Initialize() for per-execution setup
    //------------------------------------------------------------------
    DriveToPose(subsystems::CommandSwerveDrivetrain *chassis);

    //------------------------------------------------------------------
    // FRC Command Lifecycle Methods
    //------------------------------------------------------------------

    //------------------------------------------------------------------
    /// @brief      Initializes the command when scheduled
    /// @details    Called once when the command starts. Refreshes autopilot constraints,
    ///             acquires target pose(s), configures the active stage target, and publishes
    ///             state changes to notify other subsystems that navigation is active.
    /// @see        GetDriveToPoses() for target determination
    //------------------------------------------------------------------
    void Initialize() override;

    //------------------------------------------------------------------
    /// @brief      Executes the command periodically (every 20ms)
    /// @details    Updates robot pose, calls `Autopilot::Calculate()` for X/Y/heading setpoints,
    ///             applies the resulting request to the drivetrain, and logs motion/error telemetry.
    /// @see        IsFinished() for completion detection
    //------------------------------------------------------------------
    void Execute() override;

    //------------------------------------------------------------------
    /// @brief      Checks if navigation is complete
    /// @return     true if target reached or robot is stuck, false to continue
    /// @details    Returns true when the autopilot target is reached or when robot is detected
    ///             as stuck (same-pose detection). Supports midpoint-to-endpoint transition logic.
    //------------------------------------------------------------------
    bool IsFinished() override;

    //------------------------------------------------------------------
    /// @brief      Cleans up when command ends
    /// @param[in]  interrupted - true if interrupted, false if finished normally
    /// @details    Stops the robot with brake control and publishes state changes
    ///             to notify other subsystems that navigation has ended.
    //------------------------------------------------------------------
    void End(bool interrupted) override;

protected:
    //------------------------------------------------------------------
    /// @brief      Gets the target pose(s) for navigation
    /// @return     DriveToPoses struct containing endpoint and optional midpoint
    /// @details    Virtual method that derived classes override to provide
    ///             specific target pose calculations. The base implementation
    ///             returns an empty DriveToPoses struct.
    ///
    ///             **Usage Pattern:**
    ///             - Override in derived classes to provide field element targets
    ///             - Set hasMidPose=true for two-stage navigation
    ///             - Called during Initialize() to determine navigation goals
    ///
    /// @note       Derived classes should implement this to specify their targets
    /// @see        DriveOverBump::GetDriveToPoses() for multi-waypoint example
    /// @see        DriveToDepot::GetDriveToPoses() for single-target example
    //------------------------------------------------------------------
    virtual struct DriveToPoses GetDriveToPoses() { return DriveToPoses{}; }

    //------------------------------------------------------------------
    /// @brief      Updates the target pose during execution
    /// @param[in]  endPose - The new target pose to navigate to
    /// @details    Allows dynamic target changes during command execution.
    ///             Updates cached target pose and rebuilds the autopilot target with
    ///             the current stage velocity. Used for multi-stage navigation.
    /// @see        DriveOverBump::IsFinished() for example usage
    //------------------------------------------------------------------
    void SetTargetPose(const wpi::math::Pose2d &endPose);

    //------------------------------------------------------------------
    /// @brief      Sets the distance threshold for completion detection
    /// @param[in]  distanceThreshold - Distance tolerance in inches for considering target reached
    /// @details    Legacy completion distance setter retained for compatibility with
    ///             existing derived classes. Final completion now uses autopilot profile
    ///             tolerances in `Autopilot::AtTarget()`.
    ///
    /// @note       Derived classes should call this in their constructors or
    ///  initialization methods
    //------------------------------------------------------------------
    void SetDistanceThreshold(const wpi::units::length::inch_t &distanceThreshold) { m_distanceThreshold = distanceThreshold; }

    //------------------------------------------------------------------
    /// @brief      Sets the angle tolerance for completion detection
    /// @param[in]  angleTolerance - Angular tolerance in degrees for considering target rotation reached
    /// @details    Customizes how close the robot's heading must be to the target angle before
    ///             IsFinished() returns true. Smaller values provide higher rotational precision
    ///             but may increase settling time.
    ///
    /// @note       Derived classes should call this in their constructors or
    ///             initialization methods to override the default tolerance
    //------------------------------------------------------------------
    void SetAngleTolerance(const wpi::units::angle::degree_t &angleTolerance) { m_angleTolerance = angleTolerance; }
    void SetXTransitionToEndPointTolerance(const wpi::units::length::inch_t &xTolerance) { m_xtoleranceForTransitionToEndPoint = xTolerance; }
    void SetYTransitionToEndPointTolerance(const wpi::units::length::inch_t &yTolerance) { m_yToleranceForTransitionToEndPoint = yTolerance; }

    //------------------------------------------------------------------
    /// @brief      Gets the chassis subsystem pointer
    /// @return     Pointer to the swerve drive subsystem
    /// @details    Provides protected access to the chassis for derived classes
    ///             that need to query robot state or modify control behavior.
    //------------------------------------------------------------------
    subsystems::CommandSwerveDrivetrain *GetChassis() const { return m_chassis; }

    virtual wpi::units::velocity::meters_per_second_t GetMaxVelocity() const { return kMaxVelocityDefault; }
    virtual wpi::units::acceleration::meters_per_second_squared_t GetMaxAcceleration() const { return kMaxAccelerationDefault; }

    void SetIsFinished(bool isFinished) { m_isFinished = isFinished; }

private:
    bool ShouldSkipMidPoint() const;

    //------------------------------------------------------------------
    // Subsystem and Control Request Members
    //------------------------------------------------------------------

    /// @brief Pointer to the swerve drive subsystem controlled by this command
    subsystems::CommandSwerveDrivetrain *m_chassis;

    /// @brief Field-centric drive request with heading control capability
    swerve::requests::FieldCentricFacingAngle m_driveRequest;

    //------------------------------------------------------------------
    // Pose Tracking Members
    //------------------------------------------------------------------

    /// @brief Cached result of last IsFinished() evaluation, used by End() to avoid redundant recomputation
    bool m_isFinished = false;

    /// @brief Current robot pose from odometry/vision fusion
    wpi::math::Pose2d m_currentPose;

    /// @brief Target pose to navigate to
    wpi::math::Pose2d m_targetPose;
    wpi::math::Pose2d m_endPose;
    wpi::math::Pose2d m_midPose;
    bool m_hasMidPose = false;
    bool m_beforeMidPose = true;
    wpi::units::angle::degree_t m_angleTolerance{20.0};
    wpi::units::length::inch_t m_xtoleranceForTransitionToEndPoint{0.25};
    wpi::units::length::inch_t m_yToleranceForTransitionToEndPoint{0.25};
    wpi::units::velocity::meters_per_second_t m_midPointSpeed{0.0_mps};
    wpi::units::velocity::meters_per_second_t m_endPointSpeed{0.0_mps};
    wpi::units::velocity::meters_per_second_t m_currentTargetSpeed{0.0_mps};

    //------------------------------------------------------------------
    // Threshold and Range Constants
    //------------------------------------------------------------------

    /// @brief Legacy distance threshold retained for compatibility with existing derived classes.
    wpi::units::length::inch_t m_distanceThreshold{0.25};

    //------------------------------------------------------------------
    // Velocity and Acceleration Limits
    //------------------------------------------------------------------

    /// @brief Maximum translational velocity for the robot
    wpi::units::velocity::meters_per_second_t kMaxVelocity;
    static constexpr wpi::units::velocity::meters_per_second_t kMaxVelocityDefault = 4_mps;

    wpi::units::acceleration::meters_per_second_squared_t kMaxAcceleration;
    static constexpr wpi::units::acceleration::meters_per_second_squared_t kMaxAccelerationDefault = 3_mps_sq;

    double m_maxJerk = 2.0; // Maximum jerk for motion profile (m/s^3)

    //------------------------------------------------------------------
    // Runtime State Variables
    //------------------------------------------------------------------

    /// @brief Current distance error from robot to target (updated in Execute)
    wpi::units::length::meter_t m_distanceError{0.0};

    APConstraints m_autopilotConstraints{kMaxVelocityDefault, kMaxAccelerationDefault, m_maxJerk};
    APProfile m_autopilotProfile{m_autopilotConstraints};
    Autopilot m_autopilot{m_autopilotProfile};
    APTarget m_autopilotTarget{wpi::math::Pose2d{}};
};
