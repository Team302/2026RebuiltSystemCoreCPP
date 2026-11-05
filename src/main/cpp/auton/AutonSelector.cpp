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

// co-Author: notcharlie, creator of dumb code / copy paster of better code

// Includes
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <sys/stat.h>

#ifdef __linux
#include <dirent.h>
#endif

#include "wpi/nt/NetworkTableInstance.hpp"
#include "wpi/system/Filesystem.hpp"
#include "wpi/tunables/Selectable.hpp"
#include "wpi/tunables/Tunables.hpp"

// Team302 includes
#include "auton/AutonSelector.h"
#include "feedback/DriverFeedback.h"
#include "utils/FMSData.h"
#include "utils/logging/debug/Logger.h"

#include <pugixml/pugixml.hpp>

using namespace std;
using wpi::tunables::Selectable;

//---------------------------------------------------------------------
// Method: 		<<constructor>>
// Description: This creates this object and reads the auto script (CSV)
//  			files and displays a list on the dashboard.
//---------------------------------------------------------------------
AutonSelector *AutonSelector::m_instance = nullptr;
AutonSelector *AutonSelector::GetInstance()
{
	if (AutonSelector::m_instance == nullptr)
	{
		AutonSelector::m_instance = new AutonSelector();
	}
	return AutonSelector::m_instance;
}

AutonSelector::AutonSelector()
{
	PutChoicesOnDashboard();
}

string AutonSelector::GetSelectedAutoFile()
{
	std::string autonfile(wpi::filesystem::GetDeployDirectory());
	autonfile += std::filesystem::path("/auton/").string();
	autonfile += GetAlianceColor();
	autonfile += GetStartPos();
	autonfile += GetDesiredPreload();
	autonfile += GetNeutralZoneAmount();
	autonfile += GetNeutralZoneArea();
	autonfile += GetDepotOption();
	autonfile += GetOutpostOption();
	autonfile += GetFuelStrategy();
	autonfile += GetClimbingOption();
	autonfile += GetVariation();
	autonfile += std::string(".xml");

	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("auton"), string("file"), autonfile);

	auto table = wpi::nt::NetworkTableInstance::GetDefault().GetTable("auton file");

	table.get()->PutString("determined name", autonfile);

	bool fileExists = FileExists(autonfile);
	bool fileValid = FileValid(autonfile);

	DriverFeedback::GetInstance()->SetIsValidAutonFile(fileExists && fileValid);

	table.get()->PutBoolean("File Exists", fileExists);
	table.get()->PutBoolean("File Valid", fileValid);

	if (!fileExists || !fileValid)
	{
		autonfile = wpi::filesystem::GetDeployDirectory();
		autonfile += std::filesystem::path("/auton/").string();
		autonfile += GetAlianceColor();
		autonfile += ("DefaultFile.xml");
	}

	table.get()->PutString("actual file", autonfile);

	return autonfile;
}

bool AutonSelector::FileExists(const std::string &name)
{
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

bool AutonSelector::FileValid(const std::string &name)
{

	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load_file(name.c_str());
	if (result)
	{
		return true;
	}
	Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("AutonSelector"), string("FileInvalid: Description ") + name, string(result.description()));
	Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("AutonSelector"), string("FileInvalid: Offset ") + name, static_cast<int>(result.offset));
	return false;
}

string AutonSelector::GetAlianceColor()
{
	return (FMSData::GetAllianceColor() == wpi::Alliance::RED) ? std::string("Red") : std::string("Blue");
}

string AutonSelector::GetStartPos()
{
	return m_startposchooser.GetSelected();
}

string AutonSelector::GetNeutralZoneAmount()
{
	return m_neutralZoneAmount.GetSelected();
}

string AutonSelector::GetNeutralZoneArea()
{
	return m_neutralZoneArea.GetSelected();
}

string AutonSelector::GetDepotOption()
{
	return m_targetDepot.GetSelected();
}

string AutonSelector::GetOutpostOption()
{
	return m_targetOutpost.GetSelected();
}
string AutonSelector::GetClimbingOption()
{
	return m_climbing.GetSelected();
}
string AutonSelector::GetDesiredPreload()
{
	return m_desiredPreload.GetSelected();
}
string AutonSelector::GetFuelStrategy()
{
	return m_fuelStrategy.GetSelected();
}
string AutonSelector::GetVariation()
{
	return m_variation.GetSelected();
}

//---------------------------------------------------------------------
// Method: 		PutChoicesOnDashboard
// Description: This puts the list of files in the m_csvFiles attribute
//				up on the dashboard for selection.
// Returns:		void
//---------------------------------------------------------------------
void AutonSelector::PutChoicesOnDashboard()
{
	// Starting Position
	m_startposchooser.Add("Trench Depot Side", "TDep");
	m_startposchooser.Add("Bump Depot Side", "BDep");
	m_startposchooser.Add("Hub", "Hub");
	m_startposchooser.Add("Bump Outpost Side", "BOut");
	m_startposchooser.AddDefault("Trench Outpost Side", "TOut");
	wpi::tunables::Publish("StartPos", m_startposchooser);

	// Amount of times going into NZ
	m_neutralZoneAmount.Add("0", "0");
	m_neutralZoneAmount.Add("1", "1");
	m_neutralZoneAmount.Add("2", "2");
	m_neutralZoneAmount.Add("3", "3");
	m_neutralZoneAmount.Add("4", "4");
	m_neutralZoneAmount.Add("5", "5");
	m_neutralZoneAmount.AddDefault("3", "3");
	wpi::tunables::Publish("Times in Neutral Zone", m_neutralZoneAmount);

	// Area in NZ
	m_neutralZoneArea.Add("Half Field", "Half");
	m_neutralZoneArea.Add("Full Field", "Full");
	m_neutralZoneArea.Add("Both", "Combo");
	m_neutralZoneArea.Add("No Variation", "");
	m_neutralZoneArea.AddDefault("No Variation", "");
	wpi::tunables::Publish("Desired Neutral Zone Coverage", m_neutralZoneArea);

	// Depot Option
	m_targetDepot.Add("true", "Dep");
	m_targetDepot.Add("false", "NDep");
	m_targetDepot.AddDefault("false", "NDep");
	wpi::tunables::Publish("Has Depot?", m_targetDepot);

	// Outpost Option
	m_targetOutpost.Add("true", "Out");
	m_targetOutpost.Add("false", "NOut");
	m_targetOutpost.AddDefault("false", "NOut");
	wpi::tunables::Publish("Has Outpost?", m_targetOutpost);

	// Climbing Option
	m_climbing.Add("true", "Climb");
	m_climbing.Add("false", "NCli");
	m_climbing.AddDefault("false", "NCli");
	wpi::tunables::Publish("Climb?", m_climbing);

	// Preload Option
	m_desiredPreload.Add("Launch", "Launch");
	m_desiredPreload.Add("Drop", "Drop");
	m_desiredPreload.Add("Keep", "Keep");
	m_desiredPreload.AddDefault("Keep", "Keep");
	wpi::tunables::Publish("Desired Preload", m_desiredPreload);

	// Fuel Strategy Option
	m_fuelStrategy.Add("Score", "Score");
	m_fuelStrategy.Add("Pass", "Pass");
	m_fuelStrategy.AddDefault("Score", "Score");
	wpi::tunables::Publish("Desired Strategy", m_fuelStrategy);

	// Variation Option
	m_variation.Add("Winning Auton Variation", "Win");
	m_variation.Add("Losing Auton Variation", "Lose");
	m_variation.Add("Inverted Variation", "Invert");
	m_variation.Add("No Variation", "");
	m_variation.AddDefault("No Variation", "");
	wpi::tunables::Publish("Variation", m_variation);
}