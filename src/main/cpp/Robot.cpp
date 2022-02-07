// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>
#include <math.h>


using namespace std;

void Robot::RobotInit() {
  PlaceShuffleboardTiles();
  GetRobotShuffleoardValues();
  if (invertRobot)
    motorGroupRight.SetInverted(true);
  else
    motorGroupRight.SetInverted(true);
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

  // look at suffleboard...
  autonMode = frc::SmartDashboard::GetNumber("Auton Mode", 1);
  
  
}

void Robot::AutonomousPeriodic() {


  switch(autonMode) {
    case 1:
      BasicMoveAuton(); 
      break;
    default:
      BasicMoveAuton(); 
      break;
}

}

void Robot::TeleopInit() {
  
  GetTeleopShuffleBoardValues();
  pilot.setDeadzone();
  copilot.setDeadzone();
  pilot.setDeadzoneLimit(0.1);
  copilot.setDeadzoneLimit(0.1);
  pilot.setSensitivity();
  pilot.setSensitivityLevel(defaultInputSensitivity);
  
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
  

  if (pilot.GetRightStickButtonPressed())
  {
    pilot.setSensitivityLevel(driftInputSensitivity);

  }
  else if (pilot.GetRightStickButtonReleased())
  {
    pilot.setSensitivityLevel(defaultInputSensitivity);
  }

  
 

  //inputSentivityReduction = false;
  if (arcadeDrive)
  {
    std::cout << "arcadeDrive" << std::endl;
    double turningSpeed = pilot.GetLeftX() * turningSensitivity;
    double forwardSpeed = pilot.GetRightY();
    drivetrain.ArcadeDrive(forwardSpeed, turningSpeed, inputSentivityReduction);
  }
  else
  {
    //tank drive
    std::cout << "tankDrive" << std::endl;
    // if the values are close, average them
    if (abs(pilot.GetLeftY() - pilot.GetRightY()) < tankAssist)
    {
      
      //if we are using tank drive and the sticks are pretty close together, pretend that they are all the way togetehr
      double AveragePosition = Robot::Avg(pilot.GetLeftY(), pilot.GetRightY());
      //set the right stick equal to the left stick so that they are equal
      //the *-1 is because electrical refuses to let us flip the wire polarity and insisted we do it in code
      drivetrain.TankDrive(AveragePosition, AveragePosition, inputSentivityReduction);
    }
    else
    {
      drivetrain.TankDrive(pilot.GetLeftY(), pilot.GetRightY(), inputSentivityReduction);
    }
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
  frc::SmartDashboard::PutBoolean("Arcade Drive", true);
  frc::SmartDashboard::PutNumber("Tank Assist", 0.05);
  frc::SmartDashboard::PutNumber("Input Sensitivity", 0.4);
  frc::SmartDashboard::PutNumber("Turning Sensitivity", 0.6);
  frc::SmartDashboard::PutNumber("Deadzone Size", 0.05);
  frc::SmartDashboard::PutNumber("Auton Mode", 1);
  frc::SmartDashboard::PutBoolean("Invert Robot", false);
}

void Robot::GetTeleopShuffleBoardValues()
{
  //collect values from shuffleboard
  arcadeDrive = frc::SmartDashboard::GetBoolean("Arcade Drive", true);
  tankAssist = frc::SmartDashboard::GetNumber("Tank Assist", 0.08);
  defaultInputSensitivity = frc::SmartDashboard::GetNumber("Input Sensitivity", 0.4);
  turningSensitivity = frc::SmartDashboard::GetNumber("Turning Sensitivity", 0.6);
  deadzoneLimit = frc::SmartDashboard::GetNumber("Deadzone Size", 0.05);
}

void Robot::GetRobotShuffleoardValues()
{
  invertRobot = frc::SmartDashboard::GetBoolean("Invert Robot", false);
}

void Robot::BasicMoveAuton() {
  autonStep = 1;
  switch(autonStep)
  {
    case 1:
      LinearMove(10.0, 0.2);
    default:
      LinearMove(-10.0, 0.2);
  }
    
}

void Robot::LinearMove(double distance, double moterSpeed)
{
  assert (moterSpeed > 0);
  assert (distance != 0);

  double encoderDistance = InchesToEncoderTicks(distance);

  double motorFLEncoderTarget = motorFLEncoder.GetPosition() + encoderDistance;
  double motorFREncoderTarget = motorFREncoder.GetPosition() + encoderDistance;
  double motorBLEncoderTarget = motorBLEncoder.GetPosition() + encoderDistance;
  double motorBREncoderTarget = motorBREncoder.GetPosition() + encoderDistance;

  int direction;

  direction = std::copysign(direction, distance);

  if ((motorFLEncoder.GetPosition() * direction < motorFLEncoderTarget * direction) && 
        (motorFREncoder.GetPosition() * direction < motorFREncoderTarget * direction))
  {
    drivetrain.TankDrive(moterSpeed, moterSpeed, true);
  }
  else 
  {
    autonStep+=1;
  }

}

double Robot::EncoderTicksToInches(double ticks)
{
  double c = WheelRadiusInches * PI * 2;
  return (c * (ticks));
}

double Robot::InchesToEncoderTicks(double inches)
{
  double c = WheelRadiusInches * PI * 2;
  return ((inches / c)); 
}
double Robot::EncoderTicksToInches(double ticks, double TicksPerRev)
{
  double c = WheelRadiusInches * PI * 2;
  return (c * (ticks / TicksPerRev));
}

double Robot::InchesToEncoderTicks(double inches, double TicksPerRev)
{
  double c = WheelRadiusInches * PI * 2;
  return ((inches / c) * TicksPerRev); 
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif  