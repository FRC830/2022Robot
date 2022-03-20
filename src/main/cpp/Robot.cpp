// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Auton.h"
#include "Teleop.h"
#include <iostream>
#include <math.h>


using namespace std;

void Robot::RobotInit() {
  PlaceShuffleboardTiles();
  GetRobotShuffleoardValues();
  motorGroupLeft.SetInverted(true);

  frc::CameraServer::StartAutomaticCapture();

  // motorFR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // motorBR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // motorBL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // motorFL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  // Initial Auton Modes
  autonChooser.SetDefaultOption(stayAuton, stayAuton);
  autonChooser.AddOption(stayAuton, stayAuton);
  autonChooser.AddOption(taxiAuton, taxiAuton);
  autonChooser.AddOption(stayLowAuton, stayLowAuton);
  autonChooser.AddOption(stayLowTaxiAuton, stayLowTaxiAuton);
  autonChooser.AddOption(oneBallAuton, oneBallAuton);
  autonChooser.AddOption(twoBallLeftAuton, twoBallLeftAuton);
  autonChooser.AddOption(twoBallMiddleAuton, twoBallMiddleAuton);
  autonChooser.AddOption(twoBallRightAuton, twoBallRightAuton);
  autonChooser.AddOption(twoBallLineLeftAuton, twoBallLineLeftAuton);
  autonChooser.AddOption(twoBallLineRightAuton, twoBallLineRightAuton);
  autonChooser.AddOption(twoBallLineMiddleAuton, twoBallLineMiddleAuton);
  autonChooser.AddOption(mysteryMode, mysteryMode);

}

void Robot::RobotPeriodic() {}


void Robot::AutonomousInit() {

  motorFR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorBR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorBL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorFL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  firstCallToAuton = true;
  
  autonMovingMotor = false;
  autonStep = 1;
  newAutonCall = true;


  motorFLEncoder.SetPosition(0.0);
  motorFREncoder.SetPosition(0.0);
  motorBLEncoder.SetPosition(0.0);
  motorBREncoder.SetPosition(0.0);

  if (motorBREncoder.SetPosition(0.0) == rev::REVLibError::kOk)
  {
    std::cout << std::endl << "BR Encoder successfully set to: " << motorBREncoder.GetPosition() << std::endl;
  }
  std::cout << "FL Encoder " << motorFLEncoder.GetPosition() << std::endl;
  std::cout << "BL Encoder " << motorBLEncoder.GetPosition() << std::endl;
  std::cout << "FR Encoder " << motorFREncoder.GetPosition() << std::endl;

  // look at suffleboard...

  leftFlywheelTalon.Set(TalonFXControlMode::Velocity, 0);
  rightFlywheelTalon.Set(TalonFXControlMode::Follower, leftFlywheelTalon.GetDeviceID());
  backSpinTalon.Set(TalonFXControlMode::Velocity, 0);
  rightFlywheelTalon.SetInverted(true);
  backSpinTalon.SetInverted(true);

  
  leftVictor.Set(VictorSPXControlMode::PercentOutput, 0);
  middleVictor.Set(VictorSPXControlMode::PercentOutput, -0);
  rightVictor.SetInverted(true);
  rightVictor.Set(VictorSPXControlMode::Follower, leftVictor.GetDeviceID());
  
}

void Robot::AutonomousPeriodic() {

  if (firstCallToAuton)
  {
    firstCallToAuton = false;
    return;
  }


  // New Auton Selection with Sendable Chooser:

  std::string currentAutonMode = autonChooser.GetSelected();

  if (currentAutonMode == stayAuton){
    //Does nothing
  }

  else if (currentAutonMode == taxiAuton){
    switch (autonStep)
    {
      case 1:
        LinearMove(-84.75, 0.55);
      break;
      case 200: 
        std::printf("here here");
          break;
    default:
      autonStep++;
      break;
    }
  }
  else if (currentAutonMode == stayLowAuton)
  {
    switch (autonStep)
      {
        case 1:
          AccelerateFlywheelDuringAuton(3000, 2.5);
          break;
        case 110:
          RunBallManagement(0.5);
          break;
      default:
        autonStep++;
        break;
      }
  }
  else if (currentAutonMode == stayLowTaxiAuton)
  {
    switch (autonStep)
      {
        case 1:
          AccelerateFlywheelDuringAuton(3000, 2.5);
          break;
        case 100:
          RunBallManagement(0.5);
          break;
        case 150:
          LinearMove(-84.75, 0.55);
          break;
      default:
        autonStep++;
        break;
      }
  }
  else if (currentAutonMode == oneBallAuton){
    switch (autonStep)
    {
    case 1:
        LinearMove(-84.75, 0.55);
      break;
    case 2:
        AccelerateFlywheelDuringAuton(4500, 4.0);
      break;
    case 200:
        RunBallManagement(0.5);
      break;
    default:
      autonStep++;
      break;
    }
  }


  else if (currentAutonMode == twoBallLeftAuton){
  runIntake(0.9);

  switch(autonStep)
  {
    case 1:
      LinearMove(-84.75, 0.55);
      break;
    case 2:
      AccelerateFlywheelDuringAuton(4500, 4.0);
      break;
    case 100:
      RunBallManagement(0.5);
      break;
    default:
      autonStep++;
      break;
  }
}

else if (currentAutonMode == twoBallMiddleAuton){
    switch(autonStep)
  {
    case 1: 
      autonStep++;
    case 2:
      LinearMove(-67.0, 0.55);
      break;
    case 3:
      CenterPointTurn(115.0, 0.55);
      break;
    case 4:
      runIntake(0.9);
    case 5:
      LinearMove(-29.0, 0.55);
      break;
    case 6:
      CenterPointTurn(-60.0, 0.55);
      break;
    case 7: 
      AccelerateFlywheelDuringAuton(4500, 4.0);
      break;
    case 140:
      RunBallManagement(0.5);
      break;
    default:
      autonStep++;
      break;
    }
}

else if (currentAutonMode == twoBallRightAuton){
  if (autonStep > 10)
  {
    runIntake(0.5);
  }

  switch(autonStep)
  {
    case 1:
      LinearMove(-80.75, 0.55);
      break;
     case 3:
      AccelerateFlywheelDuringAuton(4500, 4.0);
      break;
    case 100:
      RunBallManagement(0.5);
      break;
    default:
      autonStep++;
      break;
  }
}
else if (currentAutonMode == twoBallLineLeftAuton){
  if (autonStep > 10)
  {
    runIntake(0.5);
  }

  switch(autonStep)
  {
    case 1:
      LinearMove(-41.0, 0.55);
      break;
     case 3:
      AccelerateFlywheelDuringAuton(4500, 4.0);
      break;
    case 120:
      RunBallManagement(0.5);
      break;
    default:
      autonStep++;
      break;
  }
}

  else if (currentAutonMode == twoBallLineMiddleAuton){
    if (autonStep >= 11)
    {
      runIntake(0.5);
    }
    switch(autonStep)
    {
      case 1: 
        autonStep++;
      case 2:
        LinearMove(-55.0, 0.55);
        break;
       break;
      case 4:
        CenterPointTurn(-14.0, 0.55);
        break;
      case 5:
        AccelerateFlywheelDuringAuton(4500, 4.0);
        break;
       break;
      case 110:
        RunBallManagement(0.5);
        break;
      default:
        autonStep++;
        break;
      }
  }

  else if (currentAutonMode == twoBallLineRightAuton)
  {
    if (autonStep > 10)
    {
      runIntake(0.5);
    }

    switch(autonStep)
    {
      case 1:
        LinearMove(-39.0, 0.55);
        break;
       break;
      case 3:
        AccelerateFlywheelDuringAuton(4500, 4.0);
        break;
      case 110:
        RunBallManagement(0.5);
        break;
      default:
        autonStep++;
        break;
    }
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

  if (ebrake)
  {
  motorFR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorBR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorBL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorFL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  }


  
}

void Robot::TeleopPeriodic() {
  GetTeleopShuffleBoardValues();

  autoAligning = pilot.GetAButton();
  if (autoAligning) {
    //KEEP THESE TWO SEPERATE IF STATEMENTS!!! VERY IMPROTANT!!!
    if (AimRobotAtHub(0.4)){
      //rumble
      pilot.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1);
      pilot.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1);
    }
  }
  else {
    autoAligning = false;
  }

  HandleDrivetrain();
  HandleShooter();
  HandleBallManagement();
  HandleIntake();
}

void Robot::DisabledInit() {
  firstCallToAuton = true;
  firstCallToAuton = true;
  autonMovingMotor = false;
  autonStep = 1;
  newAutonCall = true;

  motorFR.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  motorBR.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  motorBL.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  motorFL.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::PlaceShuffleboardTiles()
{
  //Initially place values to Shuffleboard
  frc::SmartDashboard::PutBoolean("Arcade Drive", true);
  frc::SmartDashboard::PutNumber("Tank Assist", 0.05);
  frc::SmartDashboard::PutNumber("Input Sensitivity", 0.4);
  frc::SmartDashboard::PutNumber("Turning Sensitivity", 0.91);
  frc::SmartDashboard::PutNumber("Deadzone Size", 0.05);
  frc::SmartDashboard::PutNumber("Auton Mode", 2);
  frc::SmartDashboard::PutBoolean("Invert Robot", false);

  frc::SmartDashboard::PutBoolean("Ebrake", true);
  
  //frc::SmartDashboard::PutNumber("GearRatio", gearRatio);
  
  frc::SmartDashboard::PutNumber("Shooter Maximum", 4000);
  frc::SmartDashboard::PutNumber("Shooter HANGER", 100);
  frc::SmartDashboard::PutNumber("Shooter Output", 0);
  frc::SmartDashboard::PutNumber("ratio backspin to flywheel",4);
  frc::SmartDashboard::PutNumber("Ball Management Output", 0);
  frc::SmartDashboard::PutNumber("Ball Management Maximum", 0.5);  
  frc::SmartDashboard::PutNumber("Intake Maximum", 0.5);
  frc::SmartDashboard::PutNumber("Intake Output", 0);
  frc::SmartDashboard::PutBoolean("Intake Extended", false);
  frc::SmartDashboard::PutBoolean("GET Y BUTTON", false);

  frc::SmartDashboard::PutData("Auton Modes", &autonChooser);
}

void Robot::GetTeleopShuffleBoardValues()
{
  //Collect values from Shuffleboard
  arcadeDrive = frc::SmartDashboard::GetBoolean("Arcade Drive", true);
  tankAssist = frc::SmartDashboard::GetNumber("Tank Assist", 0.08);
  defaultInputSensitivity = frc::SmartDashboard::GetNumber("Input Sensitivity", 0.4);
  turningSensitivity = frc::SmartDashboard::GetNumber("Turning Sensitivity", 0.91);
  deadzoneLimit = frc::SmartDashboard::GetNumber("Deadzone Size", 0.05);

  ebrake = frc::SmartDashboard::GetNumber("Ebrake", true);

  shooterMaximum = frc::SmartDashboard::GetNumber("Shooter Maximum", 4000);
  shooterHANGER = frc::SmartDashboard::GetNumber("Shooter HANGER MAX", 100);
  shooterOutput = frc::SmartDashboard::GetNumber("Shooter Output", 0);
  ballManageOutput = frc::SmartDashboard::GetNumber("Ball Management Output", 0);
  shooterOutput = frc::SmartDashboard::GetNumber("Ball Management Maximum",0.5);
  intakeOutput = frc::SmartDashboard::GetNumber("Intake Output", 0);
  intakeMaximum = frc::SmartDashboard::GetNumber("Intake Maximum", 0.9);
  intakeExtended = frc::SmartDashboard::GetBoolean("Intake Extended", false);
  //longSHOTHANGER = frc::SmartDashboard::GetBoolean("GET Y BUTTON", false);
}

void Robot::GetRobotShuffleoardValues()
{
  invertRobot = frc::SmartDashboard::GetBoolean("Invert Robot", false);
  frc::SmartDashboard::PutNumber("aim tolerance", 40);
  //gearRatio = frc::SmartDashboard::GetNumber("GearRatio", gearRatio);
}

void Robot::Taxi() {
  if (autonStep == 1) {
    LinearMove(-84.75, 0.5);

  }
}

void Robot::TestAuton() {
  // std::cout << "gear ratio: " << std::to_string(gearRatio) << std::endl;
  //drivetrain.TankDrive(0.3, 0.3, true);
  
  //std::printf("Basic move ton");
  //autonStep = 1;
  std::cout << "Basic Auton is running!" << std::endl;
  switch(autonStep)
  {
    case 1:
      //std::printf("In the switch");
      CenterPointTurn(90.0, 0.0009);
      break;
    case 2:
      CenterPointTurn(-90.0, 0.0009);
      break;
    default:
      break;        
  }
  return;
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif