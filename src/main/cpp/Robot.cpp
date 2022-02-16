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
  invertRobot ? 
    motorGroupRight.SetInverted(true) : 
    motorGroupRight.SetInverted(true);

  motorFLEncoder.SetPosition(0);
  motorFREncoder.SetPosition(0);
  motorBLEncoder.SetPosition(0);
  motorBREncoder.SetPosition(0);
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

  // motorFLEncoder.SetPosition(0);
  // motorFREncoder.SetPosition(0);
  // motorBLEncoder.SetPosition(0);
  // motorBREncoder.SetPosition(0);
  
  
}

void Robot::AutonomousPeriodic() {
    //drivetrain.TankDrive(0.5,0.5,false);

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
  
  frc::SmartDashboard::PutNumber("GearRatio", 3/20);
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
  gearRatio = frc::SmartDashboard::GetNumber("GearRatio", 20/3);
}

void Robot::BasicMoveAuton() {
  //drivetrain.TankDrive(0.3, 0.3, true);
  
  //std::printf("Basic move ton");
  //autonStep = 1;
  //cout << "Basic Auton is running!" << endl;
  switch(autonStep)
  {
    case 1:
      //std::printf("In the switch");
      LinearMove(-30.0, 0.2);
      break;
    case 2:
      std::printf("New move!!");
      LinearMove(30.0, 0.2);
      break;
    default:
      std::cout << "AUTON IS OVER, BOZO" << std::endl;
      break;
  }
  return;
}

void Robot::LinearMove(double distance, double motorSpeed)
{
  // std::printf("Distance: " + std::str(distance));
  // std::printf("motor Speed: " + std::str(motorSpeed));
  assert (motorSpeed > 0);
  assert (distance != 0);

  double encoderDistance = InchesToEncoderTicks(distance);
  std::cout << std::to_string(encoderDistance) << std::endl;
  if (newAutonCall)
  {
    std::cout << std::noboolalpha << newAutonCall << std::endl;
    std::printf("Here");
    
    motorFLEncoderTarget = motorFLEncoder.GetPosition() + encoderDistance;
    motorFREncoderTarget = motorFREncoder.GetPosition() + encoderDistance;
    motorBLEncoderTarget = motorBLEncoder.GetPosition() + encoderDistance;
    motorBREncoderTarget = motorBREncoder.GetPosition() + encoderDistance;
    newAutonCall = false;
    std::cout << std::noboolalpha << newAutonCall << std::endl;
  }




  int direction;
  //here
  //std::printf("in the function");
  direction = distance / abs(distance);

  std::cout << ((std::to_string(motorFLEncoder.GetPosition()) + std::to_string(motorFLEncoderTarget) +
          std::to_string(motorFREncoder.GetPosition()) +std::to_string( motorFREncoderTarget) )) << std::endl;

  if ((motorFLEncoder.GetPosition() * direction < motorFLEncoderTarget * direction) && 
        (motorFREncoder.GetPosition() * direction < motorFREncoderTarget * direction))
  {
    std::cout << "Distance to target: " << std::to_string(motorFLEncoderTarget - motorFLEncoder.GetPosition()) << std::endl;
    drivetrain.TankDrive(motorSpeed * direction, motorSpeed * direction, true);
    return;
  }
  else
  {
    std::cout << "Done with the drive" << std::endl;
    drivetrain.TankDrive(0, 0, false);
    std::cout << "auton step before: " << std::to_string(autonStep) << std::endl;
    autonStep++;
    std::cout << "auton step after: " << std::to_string(autonStep) << std::endl;
    newAutonCall = true;
    return;
  }

}

void Robot::CenterPointTurn(double degrees, double motorSpeed)
{
  // std::printf("Distance: " + std::str(distance));
  // std::printf("motor Speed: " + std::str(motorSpeed));
  assert (motorSpeed > 0);
  assert (degrees != 0);

  double distance = DegreesToInches(degrees);
  double encoderDistance = InchesToEncoderTicks(distance);

  int direction;
  //here
  //std::printf("in the function");
  direction = distance / abs(distance);

  if (newAutonCall)
  {

    std::cout << std::noboolalpha << newAutonCall << std::endl;
    std::printf("Here");
    
    motorFLEncoderTarget = motorFLEncoder.GetPosition() + encoderDistance;
    motorBLEncoderTarget = motorBLEncoder.GetPosition() + encoderDistance;
    motorFREncoderTarget = motorFREncoder.GetPosition() - encoderDistance;
    motorBREncoderTarget = motorBREncoder.GetPosition() - encoderDistance;
    newAutonCall = false;
    std::cout << std::noboolalpha << newAutonCall << std::endl;
  }

  if ((motorFLEncoder.GetPosition() * direction < motorFLEncoderTarget * direction) && 
        (motorFREncoder.GetPosition() * direction < motorFREncoderTarget * direction))
  {
    drivetrain.TankDrive(motorSpeed, motorSpeed, true);
    std::cout << "motorspeed: " << std::to_string(motorSpeed) << std::endl; 
  }
  else
  {
    std::cout << "Done with the drive" << std::endl;
    drivetrain.TankDrive(0, 0, false);
    std::cout << "auton step before: " << std::to_string(autonStep) << std::endl;
    autonStep++;
    std::cout << "auton step after: " << std::to_string(autonStep) << std::endl;
    newAutonCall = true;
    return;
  }

}

double Robot::EncoderTicksToInches(double ticks)
{
  double c = WheelRadiusInches * PI * 2;
  return (c * (ticks) * gearRatio);
}

double Robot::InchesToEncoderTicks(double inches)
{
  double c = WheelRadiusInches * PI * 2;
  return ((inches / c) * gearRatio); 
}
double Robot::EncoderTicksToInches(double ticks, double TicksPerRev)
{
  double c = WheelRadiusInches * PI * 2;
  return (c * (ticks / TicksPerRev) * gearRatio);
}

double Robot::InchesToEncoderTicks(double inches, double TicksPerRev)
{
  double c = WheelRadiusInches * PI * 2;
  return ((inches / c) * TicksPerRev * gearRatio); 
}

double Robot::DegreesToInches(double degrees)
{
  
  double RobotC = rotationAxisRadius * PI * 2;
  double radialPortion = degrees / 360;
  return (RobotC * radialPortion);
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif  