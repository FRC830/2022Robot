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

  // motorFR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // motorBR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // motorBL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // motorFL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
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

  motorFR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorBR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorBL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorFL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  firstCallToAuton = true;
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
  autonMode = frc::SmartDashboard::GetNumber("Auton Mode", 1);

  firstCallToAuton = true;

  // motorFLEncoder.SetPosition(0);
  // motorFREncoder.SetPosition(0);
  // motorBLEncoder.SetPosition(0);
  // motorBREncoder.SetPosition(0);

  
  
}

void Robot::AutonomousPeriodic() {

  if (firstCallToAuton)
  {
    firstCallToAuton = false;
    return;
  }

  std::cout << "BR Encoder " << motorFLEncoder.GetPosition() << std::endl;
  std::cout << "FL Encoder " << motorFLEncoder.GetPosition() << std::endl;
  std::cout << "BL Encoder " << motorBLEncoder.GetPosition() << std::endl;
  std::cout << "FR Encoder " << motorFREncoder.GetPosition() << std::endl;

   switch(autonMode) {
     case 1:
       Taxi(); 
       break;
     case 2:
       BackupAndShootAuton();
       break; 
     case 3:
       TestAuton();
       break; 
     default:
       Taxi(); 
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

  if (ebrake)
  {
  motorFR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorBR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorBL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorFL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  }


  
}

void Robot::TeleopPeriodic() {
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
  GetTeleopShuffleBoardValues();
  HandleDrivetrain();
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
    double turningSpeed = pilot.GetLeftX() * turningSensitivity;
    double forwardSpeed = pilot.GetRightY();
    if (autoAligning)
    {
      turningSpeed = 0;
    }
    drivetrain.ArcadeDrive(forwardSpeed, turningSpeed * -1, inputSentivityReduction);
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

bool Robot::AimRobotAtHub(double motorSpeed)
{
  //tab stands for table
  double distance = visionTab -> GetNumber("Hub Center X Distance", -2);
  std::cout << std::to_string(distance) << std::endl;
  if (distance == -1)
  {
    std::cout << "No Hub Detected" << std::endl;
    return false;
  }
  else 
  {
    std::cout << "Hub Detected" << std::endl;
  }
  
  double goal = frc::SmartDashboard::GetNumber("X resolution", 1080) / 2;
  if (abs(distance - goal) > frc::SmartDashboard::GetNumber("aim tolerance", 25))
  {
    autonStep++;
    return false;
  }

  if (distance > goal)
  {
    drivetrain.ArcadeDrive(0, motorSpeed, false);
  }
  else 
  {
    drivetrain.ArcadeDrive(0, motorSpeed * -1, false);
  }
  return true;
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
  frc::SmartDashboard::PutNumber("Auton Mode", 2);
  frc::SmartDashboard::PutBoolean("Invert Robot", false);

  frc::SmartDashboard::PutBoolean("Ebrake", true);
  
  //frc::SmartDashboard::PutNumber("GearRatio", gearRatio);
}

void Robot::GetTeleopShuffleBoardValues()
{
  //collect values from shuffleboard
  arcadeDrive = frc::SmartDashboard::GetBoolean("Arcade Drive", true);
  tankAssist = frc::SmartDashboard::GetNumber("Tank Assist", 0.08);
  defaultInputSensitivity = frc::SmartDashboard::GetNumber("Input Sensitivity", 0.4);
  turningSensitivity = frc::SmartDashboard::GetNumber("Turning Sensitivity", 0.6);
  deadzoneLimit = frc::SmartDashboard::GetNumber("Deadzone Size", 0.05);

  ebrake = frc::SmartDashboard::GetNumber("Ebrake", true);
  
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

void Robot::BackupAndShootAuton() {
  // std::cout << "gear ratio: " << std::to_string(gearRatio) << std::endl;
  //drivetrain.TankDrive(0.3, 0.3, true);
  
  //std::printf("Basic move ton");
  //autonStep = 1;
  std::cout << "Basic Auton is running!" << std::endl;
  switch(autonStep)
  {
    case 1:
      //std::printf("In the switch");
      //LinearMove(-84.75, 0.5);
      autonStep = 2;
      break;
    case 2:
      AimRobotAtHub(0.4);
      break;
    default:
      break;
  }
  return;
}

void Robot::LinearMove(double distance, double motorSpeed)
{
  assert (motorSpeed > 0);
  assert (distance != 0);

  double encoderDistance = InchesToEncoderTicks(distance);
  std::cout << "encoder distance in ticks: " << std::to_string(encoderDistance) << std::endl;
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

  // std::cout << ((std::to_string(motorFLEncoder.GetPosition()) + std::to_string(motorFLEncoderTarget) +
  //         std::to_string(motorFREncoder.GetPosition()) +std::to_string( motorFREncoderTarget) )) << std::endl;

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
//   // std::printf("Distance: " + std::str(distance));
//   // std::printf("motor Speed: " + std::str(motorSpeed));
   assert (motorSpeed > 0);
   assert (degrees != 0);

   double distance = DegreesToInches(degrees);
   double encoderDistance = InchesToEncoderTicks(distance);
    std::cout << std::to_string(encoderDistance) << std::endl;

    int direction;
    //here
    //std::printf("in the function");
    direction = distance / -1 * abs(distance);

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

    
    // The "if" continuously checks to see if we're done with the auton rotation
    // The body underneath continues to run if we're not
    // The "else" body runs once it sees that we are (and increases the autonStep)

    if ((motorFLEncoder.GetPosition() * direction * -1 < motorFLEncoderTarget * direction * -1) && 
          (motorFREncoder.GetPosition() * direction < motorFREncoderTarget * direction))
    {
      std::printf((motorFLEncoder.GetPosition() * direction * -1 < motorFLEncoderTarget * direction * -1) ? "First: True\n" : "First: False\n");
      std::printf( (motorFREncoder.GetPosition() * direction < motorFREncoderTarget * direction) ? "Second: True\n" : "Second: False\n");

      drivetrain.TankDrive(motorSpeed * direction * -1, motorSpeed * direction, true);
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
  assert(inches != 0);
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