// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"



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
  // Collect input from XBox controllers
  // collect pilot joystick values using pilot object
  pilotLeftStickX = pilot.GetLeftX();
  pilotRightStickX = pilot.GetRightX();
  pilotLeftStickY = pilot.GetLeftY();
  pilotRightStickY = pilot.GetRightY();

  // collect copilot joystick values using copilot object
  copilotLeftStickX = copilot.GetLeftX();
  copilotRightStickX = copilot.GetRightX();
  copilotLeftStickY = copilot.GetLeftY();
  copilotRightStickY = copilot.GetRightY();

  if (pilot.GetAButton() || copilot.GetAButton()) {
    printf("LoL you press A");
  }

  // Set speed of motor controller groups based on joystick values
  drivetrain.TankDrive(pilotLeftStickY, pilotRightStickY, squareInputs);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
