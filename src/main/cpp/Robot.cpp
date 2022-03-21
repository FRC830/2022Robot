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
  motorGroupLeft.SetInverted(true);

  frc::CameraServer::StartAutomaticCapture();

  // motorFR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // motorBR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // motorBL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // motorFL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  // Initial Auton Modes
  autonChooser.SetDefaultOption(stayAuton, stayAuton);
  autonChooser.AddOption(oneBallLeftAuton, oneBallLeftAuton);
  autonChooser.AddOption(oneBallRightAuton, oneBallRightAuton);
  autonChooser.AddOption(twoBallLeftAuton, twoBallLeftAuton);
  autonChooser.AddOption(twoBallRightAuton, twoBallRightAuton);
  autonChooser.AddOption(oneBallLineLeftAuton, oneBallLineLeftAuton);
  autonChooser.AddOption(oneBallLineRightAuton, oneBallLineRightAuton);
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
  autonMode = frc::SmartDashboard::GetNumber("Auton Mode", 2);

  firstCallToAuton = true;

  // motorFLEncoder.SetPosition(0);
  // motorFREncoder.SetPosition(0);
  // motorBLEncoder.SetPosition(0);
  // motorBREncoder.SetPosition(0);

  leftFlywheelTalon.Set(TalonFXControlMode::Velocity, 0);
  rightFlywheelTalon.Set(TalonFXControlMode::Follower, leftFlywheelTalon.GetDeviceID());
  backSpinTalon.Set(TalonFXControlMode::Velocity, 0);
  rightFlywheelTalon.SetInverted(true);
  backSpinTalon.SetInverted(true);

  
  
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


void Robot::HandleDrivetrain() {
  //original input sensitivity code + turbo. Uncomment if needed
  // if (pilot.GetRightStickButtonPressed())
  // {
  //   pilot.setSensitivityLevel(driftInputSensitivity);

  // }
  // else if (pilot.GetRightStickButtonReleased())
  // {
  //   pilot.setSensitivityLevel(defaultInputSensitivity);
  // }


  //new, more advanced input sensitivity. Revert to old version if this does not work.
  pilot.setSensitivityLevel(defaultInputSensitivity + (((pilot.GetRightTriggerAxis("noS") < 0.9) ? pilot.GetRightTriggerAxis("noS") : 1)  * (1 - defaultInputSensitivity)));


  //inputSentivityReduction = false;
  if (arcadeDrive)
  {
    double turningSpeed = pilot.GetLeftX() * turningSensitivity * -1;
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

void Robot::HandleShooter(){
  int dist = frc::SmartDashboard::GetNumber("Shuffleboard/vision/distance",180);
  double ratio = frc::SmartDashboard::GetNumber("ratio backspin to flywheel",3.6);


  longSHOTHANGER = copilot.GetYButton();
  frc::SmartDashboard::PutBoolean("GET Y BUTTON", longSHOTHANGER);

  //index is the index of the distance data point right above, -1 if its above everything
  int index=-1;
  for(int i =0; i< distances.size(); i++){
    if (dist<distances[i]){
      index=i;
      break;
    }

  }
  if(index !=-1 && index ==0){
    int indexabove=index;
    int indexbelow=index;

    double proportionBetweenDistancePoints=(dist-distances[indexbelow])/(distances[indexabove]-distances[indexbelow]);
    double targetRatio=((ratioMap[distances[indexabove]] - ratioMap[distances[indexbelow]])*proportionBetweenDistancePoints)+ratioMap[distances[indexbelow]];
    ratio=targetRatio;

    double targetSpeed=((speedMap[distances[indexabove]] - speedMap[distances[indexbelow]])*proportionBetweenDistancePoints)+speedMap[distances[indexbelow]];
    shooterMaximum=targetSpeed;
  }
  

  // shooterOutput = ((copilot.GetRightTriggerAxis("noS") > 0.6) && !(copilot.GetLeftTriggerAxis("noS") > 0.6)) ? shooterMaximum : shooterMaximum;
  // shooterOutput = shooterOutput == 0 && (copilot.GetLeftTriggerAxis("noS")) ? shooterMaximum * -0.5 : 0;

  //shooterOutput = ((copilot.GetRightTriggerAxis("noS") > 0.6) && !(copilot.GetLeftTriggerAxis("noS") > 0.6)) ? shooterMaximum * 1 : shooterOutput;
  //shooterOutput = copilot.GetLeftTriggerAxis("nos") > 0.6 && !copilot.GetRightTriggerAxis("noS") > 0.6 ? shooterOutput * -0.5 : shooterOutput; 
  //shooterOutput = !copilot.GetLeftTriggerAxis("nos") > 0.6 && !copilot.GetRightTriggerAxis("noS") > 0.6 ? shooterOutput : 0;

  //comment out
  //static bool lastCopilotRightTrigger = false;
  static bool risingEdgeFound = false;

  if (copilot.GetRightTriggerAxis("noS") <= 0.6)
  {
    //lastCopilotRightTrigger = true;
    // if (shooterOutput!=shooterMaximum && !risingEdgeFound){
    //   //call on the rising edge
    //   risingEdgeFound = true;
    shootStablizer = TIMERLENGTH;
    std::cout << "stopped" << std::endl;
    //   std::cout << shooterOutput << "," << shooterMaximum << std::endl;
    // }
    //shooterOutput = shooterMaximum;
    
  }



  // if ((copilot.GetRightTriggerAxis("noS") > 0.6) && (copilot.GetLeftTriggerAxis("noS") > 0.6))
  // {

  // }
  if (copilot.GetRightTriggerAxis("noS") > 0.6 && !longSHOTHANGER) 
  {
    //lastCopilotRightTrigger = true;
    // if (shooterOutput!=shooterMaximum && !risingEdgeFound){
    //   //call on the rising edge
    //   risingEdgeFound = true;
    //   shootStablizer = 20;
    //   std::cout << shooterOutput << "," << shooterMaximum << std::endl;
    // }
    shooterOutput = shooterMaximum;
    
  }
  // else if (copilot.GetLeftTriggerAxis("noS") > 0.6)
  // {
  //   shooterMaximum = shooterMaximum * -0.5;
  // }
  else if (longSHOTHANGER && copilot.GetRightTriggerAxis("nos") > 0.6) {
    shooterOutput = shooterHANGER;
  }
  if (copilot.GetLeftTriggerAxis("noS") < 0.2)
  {
    risingEdgeFound = false;
  }
  //Apply Ryan's confusing Deadzone math:
  //The following line serves as a deadzone maximum ex: 0.7- (0.7-0.6)
  shooterOutput = longSHOTHANGER ? shooterHANGER-Deadzone(shooterHANGER-shooterOutput) : shooterMaximum - Deadzone(shooterMaximum - shooterOutput);

  if (shooterOutput > 200)
  {
    leftFlywheelTalon.Set(TalonFXControlMode::Velocity, shooterOutput);
    rightFlywheelTalon.Set(TalonFXControlMode::Follower, leftFlywheelTalon.GetDeviceID());
    backSpinTalon.Set(TalonFXControlMode::Velocity, shooterOutput * ratio);
    rightFlywheelTalon.SetInverted(true);
    backSpinTalon.SetInverted(true);
  }
  else {
    leftFlywheelTalon.Set(TalonFXControlMode::PercentOutput, 0);
    rightFlywheelTalon.Set(TalonFXControlMode::Follower, leftFlywheelTalon.GetDeviceID());
    backSpinTalon.Set(TalonFXControlMode::PercentOutput, 0);
    rightFlywheelTalon.SetInverted(true);
    backSpinTalon.SetInverted(true);
  }
  

  frc::SmartDashboard::PutNumber("closed loop error", leftFlywheelTalon.GetClosedLoopError());


  //Change this to be much much much much slower!!
}

void Robot::HandleBallManagement(){
  std::cout << std::to_string(shootStablizer) << std::endl; 
  shootStablizer--;

  // leftVictor.Set(VictorSPXControlMode::Follower, leftFlywheelTalon.GetDeviceID());
  // middleVictor.Set(VictorSPXControlMode::Follower, leftFlywheelTalon.GetDeviceID());
  // rightVictor.SetInverted(true);
  // rightVictor.Set(VictorSPXControlMode::Follower, leftFlywheelTalon.GetDeviceID());

  // ballManageOutput = (copilot.GetAButton() && abs(leftFlywheelTalon.GetClosedLoopError() < 145)) ? frc::SmartDashboard::GetNumber("Ball Management Maximum", 0.5) : 0;
  ballManageOutput = (copilot.GetXButton() || (copilot.GetAButton() && (shootStablizer < 0 || shootStablizer == TIMERLENGTH))) ? frc::SmartDashboard::GetNumber("Ball Management Maximum", 0.5) : 0;
 

  bool ballManageReverse = copilot.GetBButton();

  
  //ballManageOutput = ballManageMaximum-Deadzone(ballManageMaximum-ballManageOutput);

  if (ballManageOutput > 0)
  {
    leftVictor.Set(VictorSPXControlMode::PercentOutput, ballManageOutput);
    middleVictor.Set(VictorSPXControlMode::PercentOutput, -ballManageOutput);
    rightVictor.SetInverted(true);
    rightVictor.Set(VictorSPXControlMode::Follower, leftVictor.GetDeviceID());
    ballManageReverse = false;
  }
  else if (ballManageReverse) {
    leftVictor.Set(VictorSPXControlMode::PercentOutput, -1 * frc::SmartDashboard::GetNumber("Ball Management Maximum", 0.5) );
    middleVictor.Set(VictorSPXControlMode::PercentOutput, frc::SmartDashboard::GetNumber("Ball Management Maximum", 0.5));
    rightVictor.Set(VictorSPXControlMode::Follower, leftVictor.GetDeviceID());
  }
  else {
    leftVictor.Set(VictorSPXControlMode::PercentOutput, 0);
    middleVictor.Set(VictorSPXControlMode::PercentOutput, 0);
    rightVictor.Set(VictorSPXControlMode::Follower, leftVictor.GetDeviceID());
  }

  if (pilot.GetAButtonPressed())
  {
    frc::SmartDashboard::PutNumber("\nerror at shoot: ", leftFlywheelTalon.GetClosedLoopError());
  }
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
  intakeMaximum = frc::SmartDashboard::GetNumber("Intake Maximum", 0.5);
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

void Robot::BackupAndShootAuton() {
  frc::SmartDashboard::PutNumber("closed loop error", leftFlywheelTalon.GetClosedLoopError());
  // std::cout << "gear ratio: " << std::to_string(gearRatio) << std::endl;
  //drivetrain.TankDrive(0.3, 0.3, true);
  
  //std::printf("Basic move ton");
  //autonStep = 1;
  std::cout << "Auton Step is : " << std::to_string(autonStep) << std::endl;

  runIntake(0.5);
  
  switch(autonStep)
  {
    case 1:
      LinearMove(-84.75, 0.55);
      break;
     case 10:
      AccelerateFlywheelDuringAuton(4500, 4.0);
      break;
    case 110:
      RunBallManagement(0.5);
      break;
    // case 220:
    //   RunBallManagement(0);
    //   break;
    // case 350:
    //   RunBallManagement(0.5);
    // break;
    default:
      autonStep++;
      break;
  }
  return;
}

void Robot::runIntake(double speed)
{
  intakeOutput = speed * intakeMaximum;
  intakeMotor.Set(VictorSPXControlMode::PercentOutput, -intakeOutput);
  frc::SmartDashboard::PutNumber("Intake Output", intakeOutput);

  leftSolenoid.Set(true);
  rightSolenoid.Set(true);
  frc::SmartDashboard::PutBoolean("Intake Extended", true);
}

void Robot::AccelerateFlywheelDuringAuton(int speed, double ratio)
{
  leftFlywheelTalon.Set(TalonFXControlMode::Velocity, speed);
  rightFlywheelTalon.Set(TalonFXControlMode::Follower, leftFlywheelTalon.GetDeviceID());
  backSpinTalon.Set(TalonFXControlMode::Velocity, speed * ratio);
  rightFlywheelTalon.SetInverted(true);
  backSpinTalon.SetInverted(true);

  autonStep++;
  std::cout << std::to_string(leftFlywheelTalon.GetClosedLoopError()) << std::endl;
}

void Robot::RunBallManagement(double speed)
{
  leftVictor.Set(VictorSPXControlMode::PercentOutput, speed);
  middleVictor.Set(VictorSPXControlMode::PercentOutput, -speed);
  rightVictor.SetInverted(true);
  rightVictor.Set(VictorSPXControlMode::Follower, leftVictor.GetDeviceID());
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

  // New Auton Selection with Sendable Chooser:

  std::string currentAutonMode = autonChooser.GetSelected();
  if (currentAutonMode == stayAuton){

  }
  else if (currentAutonMode == oneBallLineLeftAuton){

  }
  else if (currentAutonMode == oneBallRightAuton){

  }
  else if (currentAutonMode == twoBallLeftAuton){

  }
  else if (currentAutonMode == twoBallRightAuton){
    
  }
  else if (currentAutonMode == oneBallLineLeftAuton){

  }
  else if (currentAutonMode == oneBallLineRightAuton){
    
  }


}


void Robot::Wait_Auton(int seconds) {

    

}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif