// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>


using namespace std;

void Robot::RobotInit() {
  frc::SmartDashboard::PutBoolean("ARCADE DRIVE", true);
  tankAssist = frc::SmartDashboard::GetNumber("Tank Assist", 0.08);
  defaultinputSensitivity = frc::SmartDashboard::PutNumber("input sensitivity", 0.4);
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
  
  
  
}

void Robot::TeleopPeriodic() {
  HandleDrivetrain();
  HandleSolenoids();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}



void Robot::HandleDrivetrain() {
  //collect values from shuffleboard
  arcadeDrive = frc::SmartDashboard::GetBoolean("ARCADE DRIVE", true);
  tankAssist = frc::SmartDashboard::GetNumber("Tank Assist", 0.08);
  defaultinputSensitivity = frc::SmartDashboard::GetNumber("input sensitivity", 0.4);
  turningSensitivity = frc::SmartDashboard::GetNumber("TurningSensitivity", 0.6);

  // Collect input from XBox controllers
  // collect pilot joystick values using pilot object
  // pilotLeftStickX = pilot.GetLeftX();
  // pilotRightStickX = pilot.GetRightX();
  bool drift = false;//(pilot.GetRightStickButtonPressed() || (!pilot.GetRightStickButtonReleased()));
  
  if (drift)
  {
    std::cout << "drift!!" << std::endl;
    pilotLeftStickY = pilot.GetLeftY() * driftInputSensitivity;
    pilotRightStickY = pilot.GetRightY() * driftInputSensitivity;
    pilotLeftStickX = pilot.GetLeftX() * driftInputSensitivity;
    pilotRightStickX = pilot.GetRightX() * driftInputSensitivity;
  } 
  else
  {
    pilotLeftStickY = pilot.GetLeftY() * defaultinputSensitivity;
    pilotRightStickY = pilot.GetRightY() * defaultinputSensitivity;
    pilotLeftStickX = pilot.GetLeftX() * defaultinputSensitivity;
    pilotRightStickX = pilot.GetRightX() * defaultinputSensitivity;
  } 

  //determine if the right stick has been pressed or not released 




  
  // collect copilot joystick values using copilot object
  // Currently not using copilot values
  /*
  copilotLeftStickX = copilot.GetLeftX();
  copilotRightStickX = copilot.GetRightX();
  copilotLeftStickY = copilot.GetLeftY();
  copilotRightStickY = copilot.GetRightY();
  */  

  // TODO: make a something that determines when to set squareInputs to true, instead of always true

  //uncomment to disable square inputs
  //inputSentivityReduction = false;
  if (arcadeDrive)
  {
    std::cout << "arcadeDrive" << std::endl;
    pilotLeftStickX = Robot::Deadzone(pilotLeftStickX) * -1 * turningSensitivity;
    pilotRightStickY = Robot::Deadzone(pilotRightStickY) * -1;
    drivetrain.ArcadeDrive(pilotLeftStickX, pilotRightStickY, inputSentivityReduction);
  }
  else
  {
    //tank drive
    std::cout << "tankDrive" << std::endl;
    // if the values are close, average them
    if (abs(pilotLeftStickX - pilotRightStickY) < 0.08)
    {
      
      pilotLeftStickY = Robot::Deadzone(pilotLeftStickY);
      pilotRightStickY = Robot::Deadzone(pilotRightStickY);
      pilotLeftStickY = (pilotLeftStickY + pilotLeftStickY) / 2;
      pilotRightStickY = pilotLeftStickY * -1;
    }
    else
    {
      pilotLeftStickY = Robot::Deadzone(pilotLeftStickY);
      pilotRightStickY = Robot::Deadzone(pilotRightStickY) * -1;
    }
    drivetrain.TankDrive(pilotLeftStickY, pilotRightStickY, inputSentivityReduction);
  }
}


void Robot::HandleSolenoids() {
  if (copilot.GetBButton() == 1){
    doubleSolenoid.Set(frc::DoubleSolenoid::kForward);
  }
  else{
    doubleSolenoid.Set(frc::DoubleSolenoid::kReverse);
  }

}

double Robot::Deadzone(double amm){
    //deadzoneLimit is arbitrary
    if (abs(amm) < 0.1) 
    {
      amm = 0;
    }
    
    return amm;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif  