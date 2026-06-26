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

#include "mechanisms/launcher/LauncherContainer.h"

// FRC Includes
#include "wpi/commands2/CommandScheduler.hpp"
#include "wpi/commands2/button/Trigger.hpp"
#include "wpi/framework/RobotBase.hpp"

// Team 302 Includes
#include "mechanisms/MechanismTypes.h"
#include "mechanisms/configs/MechanismConfig.h"
#include "mechanisms/configs/MechanismConfigMgr.h"
#include "mechanisms/launcher/Launcher.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/logging/debug/Logger.h"

LauncherContainer *LauncherContainer::m_instance = nullptr;
LauncherContainer *LauncherContainer::GetInstance()
{
    if (LauncherContainer::m_instance == nullptr)
    {
        LauncherContainer::m_instance = new LauncherContainer();
    }
    return LauncherContainer::m_instance;
}

LauncherContainer::LauncherContainer()
{
}

void LauncherContainer::ConfigureBindings()
{
    auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();
    if (config == nullptr)
    {
        return;
    }

    auto mech = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::LAUNCHER);
    m_launcher = (mech != nullptr) ? dynamic_cast<Launcher *>(mech) : nullptr;
    if (m_launcher == nullptr)
    {
        // Launcher not present on this robot (commented out in the per-robot config) - nothing to bind.
        return;
    }

    auto controller = TeleopControl::GetInstance();
    if (controller == nullptr)
    {
        return;
    }

    m_launcher->SetDefaultCommand(m_launcher->GetIdleCommand().IgnoringDisable(true));

    Launcher *launcher = m_launcher;

    // NOTE on WhileTrue semantics: a Trigger only SCHEDULES on a false->true edge and only CANCELS on a
    // true->false edge. So unlike the old StateMgr (which polled an "enter this state" condition against
    // the CURRENT state), every condition below is written as a "should I be running this command right
    // now" HOLDING condition. That makes the edges line up: when the desired command changes, the old
    // command's condition falls (cancel) and the new command's condition rises (schedule) in the same loop.

    // --- Off (protected mode) -------------------------------------------------------------------------
    // Highest priority. Every other condition is gated on !IsLauncherInProtectedMode() so Off always wins.
    wpi::cmd::Trigger off([launcher]()
                          { return launcher->IsLauncherInProtectedMode(); });
    off.WhileTrue(m_launcher->GetOffCommand().IgnoringDisable(true));

    // Cold-start edge for OFF->INITIALIZE (already true at bind time, so it needs a one-shot kick).
    if (!m_launcher->IsLauncherInProtectedMode() && !m_launcher->IsLauncherInitialized())
    {
        wpi::cmd::CommandScheduler::GetInstance().Schedule(m_launcher->GetInitializeCommand().IgnoringDisable(true));
    }

    // --- Manual launch --------------------------------------------------------------------------------
    // Old ManualLaunchState: enter while the MANUAL_LAUNCH button is held; leave (to Idle) when released.
    wpi::cmd::Trigger manualLaunch([launcher]()
                                   { return !launcher->IsLauncherInProtectedMode() &&
                                            TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::MANUAL_LAUNCH); });
    manualLaunch.WhileTrue(m_launcher->GetManualLaunchCommand());

    // --- Launcher tuning ------------------------------------------------------------------------------
    // Old LauncherTuningState: stay while IsTuningLauncherMode(). It hands off to Launch when the override
    // launch button is pressed (LAUNCH_OVERRIDE && !EXTENDER_MODIFIER), so exclude that case here - when
    // the override is pressed the tuning condition falls (cancel tuning) and the launch condition rises.
    wpi::cmd::Trigger tuning([launcher]()
                             {
        auto tc = TeleopControl::GetInstance();
        bool overrideToLaunch = tc->IsButtonPressed(TeleopControlFunctions::LAUNCH_OVERRIDE) &&
                                !tc->IsButtonPressed(TeleopControlFunctions::EXTENDER_MODIFIER);
        return !launcher->IsLauncherInProtectedMode() &&
               launcher->IsTuningLauncherMode() &&
               !overrideToLaunch; });
    tuning.WhileTrue(m_launcher->GetLauncherTuningCommand());

    // --- Prepare to launch ----------------------------------------------------------------------------
    // Normal operation (not tuning): hold the LAUNCH button to spin up in PrepareToLaunch until we are at
    // target. The !IsLauncherAtTarget() term is what sends us BACK to Prepare from Launch if we fall off
    // target while still holding LAUNCH (this rises again, interrupting Launch).
    wpi::cmd::Trigger prepare([launcher]()
                              { return !launcher->IsLauncherInProtectedMode() &&
                                       !launcher->IsTuningLauncherMode() &&
                                       TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::LAUNCH) &&
                                       !launcher->IsLauncherAtTarget(); });
    prepare.WhileTrue(m_launcher->GetPrepareToLaunchCommand());

    // --- Launch ---------------------------------------------------------------------------------------
    // Two ways in, matching the old LaunchState:
    //   1. Normal: holding LAUNCH (not tuning) once we ARE at target -> mutually exclusive with Prepare.
    //   2. Override: LAUNCH_OVERRIDE && !EXTENDER_MODIFIER -> the path out of LauncherTuning, also forces a
    //      launch from PrepareToLaunch regardless of at-target.
    // Bound with OnTrue (NOT WhileTrue): once scheduled, Launch is NOT cancelled when this condition goes
    // false. It is self-governing - it runs until LauncherLaunchCommand::IsFinished() returns true (launch
    // buttons released for the debounce window, or auton launch detector) or until another command
    // (Prepare when we fall off target while still holding LAUNCH, or Off) interrupts by requiring the
    // subsystem. This is what prevents bouncing straight back to Idle the instant the condition drops.
    wpi::cmd::Trigger launch([launcher]()
                             {
        auto tc = TeleopControl::GetInstance();
        bool overrideToLaunch = tc->IsButtonPressed(TeleopControlFunctions::LAUNCH_OVERRIDE) &&
                                !tc->IsButtonPressed(TeleopControlFunctions::EXTENDER_MODIFIER);
        bool autoLaunch = !launcher->IsTuningLauncherMode() &&
                          tc->IsButtonPressed(TeleopControlFunctions::LAUNCH) &&
                          launcher->IsLauncherAtTarget();
        return !launcher->IsLauncherInProtectedMode() && (autoLaunch || overrideToLaunch); });
    launch.OnTrue(m_launcher->GetLaunchCommand());

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("LauncherContainer"), std::string("Configured"), std::string("Launcher"));
}
