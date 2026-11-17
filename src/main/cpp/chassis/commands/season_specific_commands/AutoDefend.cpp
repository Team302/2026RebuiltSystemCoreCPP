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

#include "chassis/commands/season_specific_commands/AutoDefend.h"
#include "vision/DragonVision.h"
#include "vision/PoseOffsetUtils.h"

AutoDefend::AutoDefend(subsystems::CommandSwerveDrivetrain *chassis) : wpi::cmd::CommandHelper<VisionDrive, AutoDefend>(chassis)
{
    m_vision = DragonVision::GetDragonVision();
    if (m_vision != nullptr)
    {
        m_vision->SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE::OBJECT_DETECTION, DRAGON_LIMELIGHT_PIPELINE::BUMPERS);
    }
}

std::optional<APTarget> AutoDefend::GetAutopilotTarget()
{
    if (m_vision == nullptr)
    {
        return std::nullopt;
    }
    m_vision->SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE::OBJECT_DETECTION, DRAGON_LIMELIGHT_PIPELINE::BUMPERS);

    m_visionCache = m_vision->GetObjectDetectionTargetInfo(VisionTargetOption::CLOSEST_VALID_TARGET, std::vector<int>{});
    if (!m_visionCache.empty() && m_visionCache[0].get() != nullptr)
    {
        auto targetinfo = PoseOffsetUtils::CalculateXYDistanceFromObject(*m_visionCache[0], 4_in);
        wpi::units::length::meter_t xDist = -targetinfo.first;
        xDist = std::clamp(xDist, -m_XdistLimit, 0_m);
        xDist += m_IntakeXOffset;

        wpi::units::length::meter_t yDist = -targetinfo.second;
        yDist = std::clamp(yDist, -m_YdistLimit, m_YdistLimit);

        auto currentPose = m_chassis->GetPose();
        wpi::math::Translation2d robotRelativeOffset{-xDist, -yDist};
        auto fieldRelativeOffset = robotRelativeOffset.RotateBy(currentPose.Rotation());
        wpi::math::Pose2d targetPose{currentPose.Translation() + fieldRelativeOffset, currentPose.Rotation()};

        return APTarget(targetPose);
    }
    else
    {
        return std::nullopt;
    }
}
