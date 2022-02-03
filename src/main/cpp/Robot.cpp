// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>


using namespace std;

void Robot::RobotInit() {
  PlaceShuffleboardTiles();
}

/*
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */

void Robot::RobotPeriodic() {}


void Robot::AutonomousInit() {
  
}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
  
  GetTeleopShuffleBoardValues();
  
}

void Robot::TeleopPeriodic() {
  GetTeleopShuffleBoardValues();
  HandleDrivetrain();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}


void Robot::HandleDrivetrain() {
  bool drift;

  if (pilot.GetRightStickButtonPressed())
  {
    drift = true;
  }
  else if (pilot.GetRightStickButtonReleased())
  {
    drift = false;
  }

  double adjustedPilotLeftStickY;
  double adjustedPilotRightStickY;
  double adjustedPilotLeftStickX;
  double adjustedPilotRightStickX;
  
  if (drift)
  {
    std::cout << "drift!!" << std::endl;
    adjustedPilotLeftStickY = pilot.GetLeftX() * driftInputSensitivity;
    adjustedPilotRightStickY = pilot.GetRightY() * driftInputSensitivity;
    adjustedPilotLeftStickX = pilot.GetLeftY() * driftInputSensitivity;
    adjustedPilotRightStickX = pilot.GetRightX() * driftInputSensitivity;
  } 
  else
  {
    adjustedPilotLeftStickY = pilot.GetLeftX() * defaultinputSensitivity;
    adjustedPilotRightStickY = pilot.GetRightY() * defaultinputSensitivity;
    adjustedPilotLeftStickX = pilot.GetLeftY() * defaultinputSensitivity;
    adjustedPilotRightStickX = pilot.GetRightX() * defaultinputSensitivity;
  } 

  //inputSentivityReduction = false;
  if (arcadeDrive)
  {
    std::cout << "arcadeDrive" << std::endl;
    adjustedPilotLeftStickX = adjustedPilotLeftStickX * -1 * turningSensitivity;
    adjustedPilotRightStickY = adjustedPilotRightStickY * -1;
    drivetrain.ArcadeDrive(adjustedPilotLeftStickX, adjustedPilotRightStickY, inputSentivityReduction);
  }
  else
  {
    //tank drive
    std::cout << "tankDrive" << std::endl;
    // if the values are close, average them
    if (abs(adjustedPilotLeftStickX - adjustedPilotRightStickY) < tankAssist)
    {
      
      adjustedPilotLeftStickY = adjustedPilotLeftStickY;
      adjustedPilotRightStickY = adjustedPilotRightStickY;
      
      //if we are using tank drive and the sticks are pretty close together, pretend that they are all the way togetehr
      adjustedPilotLeftStickY = Robot::Avg(adjustedPilotLeftStickY, adjustedPilotRightStickY);
      //set the right stick equal to the left stick so that they are equal
      //the *-1 is because electrical refuses to let us flip the wire polarity and insisted we do it in code
      adjustedPilotRightStickY = adjustedPilotLeftStickY * -1;
    }
    else
    {
      adjustedPilotLeftStickY = adjustedPilotLeftStickY;
      adjustedPilotRightStickY = adjustedPilotRightStickY * -1;
    }
    drivetrain.TankDrive(adjustedPilotLeftStickY, adjustedPilotRightStickY, inputSentivityReduction);
  }

}

double Robot::Deadzone(double amm){
    //deadzoneLimit is arbitrary
    //make it smartdashboard
    if (abs(amm) < deadzoneLimit) 
    {
      amm = 0;
    }
    
    return amm;
}

double Robot::Avg(double val1, double val2)
{
  return (val1 + val2) / 2;
}

void Robot::PlaceShuffleboardTiles()
{
  frc::SmartDashboard::PutBoolean("ARCADE DRIVE", true);
  frc::SmartDashboard::PutNumber("Tank Assist", 0.05);
  frc::SmartDashboard::PutNumber("input sensitivity", 0.4);
  frc::SmartDashboard::PutNumber("TurningSensitivity", 0.6);
  frc::SmartDashboard::PutNumber("Deadzone Size", 0.05);
}

void Robot::GetTeleopShuffleBoardValues()
{
  //collect values from shuffleboard
  arcadeDrive = frc::SmartDashboard::GetBoolean("ARCADE DRIVE", true);
  tankAssist = frc::SmartDashboard::GetNumber("Tank Assist", 0.08);
  defaultinputSensitivity = frc::SmartDashboard::GetNumber("input sensitivity", 0.4);
  turningSensitivity = frc::SmartDashboard::GetNumber("TurningSensitivity", 0.6);
  deadzoneLimit = frc::SmartDashboard::GetNumber("Deadzone Size", 0.05);
}



#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif  