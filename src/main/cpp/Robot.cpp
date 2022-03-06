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
  HandleShooter();
  HandleBallManagement();
  HandleIntake();
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
    double turningSpeed = pilot.GetLeftX() * -1 * turningSensitivity;
    double forwardSpeed = pilot.GetRightY() * -1;
    drivetrain.ArcadeDrive(turningSpeed, forwardSpeed, inputSentivityReduction);
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
      drivetrain.TankDrive(AveragePosition, AveragePosition*-1, inputSentivityReduction);
    }
    else
    {
      drivetrain.TankDrive(pilot.GetLeftY(), pilot.GetRightY()*-1, inputSentivityReduction);
    }
  }
}

void Robot::HandleShooter(){

  shooterOutput = copilot.GetRightTriggerAxis("noS")*shooterMaximum;
  //Apply Ryan's confusing Deadzone math:
  //The following line serves as a deadzone maximum ex: 0.7- (0.7-0.6)
  shooterOutput = shooterMaximum-Deadzone(shooterMaximum-shooterOutput);
  
  leftFlywheelTalon.Set(TalonFXControlMode::Velocity, shooterOutput);
  rightFlywheelTalon.Set(TalonFXControlMode::Follower, leftFlywheelTalon.GetDeviceID());
  backSpinTalon.Set(TalonFXControlMode::Follower, leftFlywheelTalon.GetDeviceID());
  rightFlywheelTalon.SetInverted(true);
  backSpinTalon.SetInverted(true);
}

void Robot::HandleBallManagement(){

  ballManageOutput = ((copilot.GetLeftTriggerAxis("noS")*ballManageMaximum) > .9) ? 1 : 0;
  ballManageOutput = ballManageMaximum-Deadzone(ballManageMaximum-ballManageOutput);
  
  leftVictor.Set(VictorSPXControlMode::Velocity, ballManageOutput);
  middleVictor.Set(VictorSPXControlMode::Velocity, -ballManageOutput);
  rightVictor.SetInverted(true);
  rightVictor.Set(VictorSPXControlMode::Follower, leftVictor.GetDeviceID());
}

void Robot::HandleIntake(){

  bool isIntaking = pilot.GetLeftTriggerAxis() > 0.2;
  intakeOutput = int(isIntaking) * intakeMaximum;
  intakeMotor.Set(VictorSPXControlMode::PercentOutput, -intakeOutput);
  frc::SmartDashboard::PutNumber("Intake Output", intakeOutput);

 if (isIntaking){
    leftSolenoid.Set(true);
    rightSolenoid.Set(true);
    frc::SmartDashboard::PutBoolean("Intake Extended", true);
  }
  else{
    leftSolenoid.Set(false);
    rightSolenoid.Set(false);
    frc::SmartDashboard::PutBoolean("Intake Extended", false);
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
  //Initially place values to Shuffleboard
  frc::SmartDashboard::PutBoolean("Arcade Drive", true);
  frc::SmartDashboard::PutNumber("Tank Assist", 0.05);
  frc::SmartDashboard::PutNumber("Input Sensitivity", 0.4);
  frc::SmartDashboard::PutNumber("Turning Sensitivity", 0.6);
  frc::SmartDashboard::PutNumber("Deadzone Size", 0.05);
  
  frc::SmartDashboard::PutNumber("Shooter Maximum", 0);
  frc::SmartDashboard::PutNumber("Shooter Output", 0);
  frc::SmartDashboard::PutNumber("Ball Management Output", 0);
  frc::SmartDashboard::PutNumber("Ball Management Maximum", 0);
  frc::SmartDashboard::PutNumber("Intake Maximum", 0.5);
  frc::SmartDashboard::PutNumber("Intake Output", 0);
  frc::SmartDashboard::PutBoolean("Intake Extended", false);
}

void Robot::GetTeleopShuffleBoardValues()
{
  //Collect values from Shuffleboard
  arcadeDrive = frc::SmartDashboard::GetBoolean("Arcade Drive", true);
  tankAssist = frc::SmartDashboard::GetNumber("Tank Assist", 0.08);
  defaultInputSensitivity = frc::SmartDashboard::GetNumber("Input Sensitivity", 0.4);
  turningSensitivity = frc::SmartDashboard::GetNumber("Turning Sensitivity", 0.6);
  deadzoneLimit = frc::SmartDashboard::GetNumber("Deadzone Size", 0.05);

  shooterMaximum = frc::SmartDashboard::GetNumber("Shooter Maximum", 0);
  shooterOutput = frc::SmartDashboard::GetNumber("Shooter Output", 0);
  ballManageOutput = frc::SmartDashboard::GetNumber("Ball Management Output", 0);
  shooterOutput = frc::SmartDashboard::GetNumber("Ball Management Maximum", 0);
  intakeOutput = frc::SmartDashboard::GetNumber("Intake Output", 0);
  intakeMaximum = frc::SmartDashboard::GetNumber("Intake Maximum", 0.5);
  intakeExtended = frc::SmartDashboard::GetBoolean("Intake Extended", false);
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif