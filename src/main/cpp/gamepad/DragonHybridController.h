
#pragma once

#include "gamepad/DragonXBox.h"
#include "wpi/commands2/button/CommandNiDsXboxController.hpp"

class DragonHybridController
{
public:
    DragonHybridController(int port);

    // Accessors for command-based functionality
    wpi::cmd::CommandNiDsXboxController *GetCommandController();

    // Accessors for non-command-based functionality
    DragonXBox *GetNonCommandController();

private:
    wpi::cmd::CommandNiDsXboxController *m_commandController;
    DragonXBox *m_nonCommandController;
};