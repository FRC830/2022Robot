// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

using namespace std;

void Robot::RobotInit() {}

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

  
  
}

void Robot::TeleopPeriodic() {
  HandleDrivetrain();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}



void Robot::HandleDrivetrain() {
  // Collect input from XBox controllers
  // collect pilot joystick values using pilot object
  // pilotLeftStickX = pilot.GetLeftX();
  // pilotRightStickX = pilot.GetRightX();
  pilotLeftStickY = pilot.GetLeftY();
  pilotRightStickY = pilot.GetRightY();

  
  // collect copilot joystick values using copilot object
  // Currently not using copilot values
  /*
  copilotLeftStickX = copilot.GetLeftX();
  copilotRightStickX = copilot.GetRightX();
  copilotLeftStickY = copilot.GetLeftY();
  copilotRightStickY = copilot.GetRightY();
  */  

  // TODO: make a something that determines when to set squareInputs to true, instead of always true

  squareInputs = true;
  pilotLeftStickY = Robot::Deadzone(pilotLeftStickY);
  pilotRightStickY = Robot::Deadzone(pilotRightStickY);
  drivetrain.TankDrive(pilotLeftStickY, pilotRightStickY, squareInputs);
}

void Robot::Deadzone(pilotStickY){
    //deadzoneLimit is arbitrary
    if (abs(pilotStickY) > deadzoneLimit){
      pilotStickY = 0;
    }
    return pilotStickY;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif  