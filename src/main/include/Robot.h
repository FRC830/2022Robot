// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ModifiableController.h>
#include <rev/CANSparkMax.h>
#include <string>
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedController.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <cmath> 

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define PI 3.14159265


const double deadZoneLimit = 0.05; 

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  //Handle spam below
  void HandleDrivetrain();
  //General util stuff
  void PlaceShuffleboardTiles();
  void GetTeleopShuffleBoardValues();
  void GetRobotShuffleoardValues();
  void GetControllerInput();
  double Deadzone(double pilotStickY);
  double Avg(double val1, double val2);
  double EncoderTicksToInches(double ticks, double TicksPerRev);
  double InchesToEncoderTicks(double inches, double TicksPerRev);
  double EncoderTicksToInches(double ticks);
  double InchesToEncoderTicks(double inches);
  double DegreesToInches(double degrees);
  //Auton functions... for auton...
  void LinearMove(double distance, double motorSpeed);
  void CenterPointTurn(double degrees, double motorSpeed);
  bool AimRobotAtHub(double moterSpeed);
  void CompoundMove(double distance, double degrees, double motorSpeed);
  //Auton Comb...
  void Taxi();
  void BackupAndShootAuton();
  void TestAuton();




  /*=============
  Pins & IDs
  =============*/




  // The line below is just an example, it does not contain the correct deviceID, may not contain
  // correct MotorType: Brushless
  rev::CANSparkMax motorFL = rev::CANSparkMax(1, rev::CANSparkMaxLowLevel::MotorType::kBrushless); 
  rev::CANSparkMax motorFR = rev::CANSparkMax(3, rev::CANSparkMaxLowLevel::MotorType::kBrushless); 
  rev::CANSparkMax motorBL = rev::CANSparkMax(2, rev::CANSparkMaxLowLevel::MotorType::kBrushless); 
  rev::CANSparkMax motorBR = rev::CANSparkMax(4, rev::CANSparkMaxLowLevel::MotorType::kBrushless); 
  rev::SparkMaxRelativeEncoder motorFLEncoder = motorFL.GetEncoder();
  rev::SparkMaxRelativeEncoder motorFREncoder = motorFR.GetEncoder();
  rev::SparkMaxRelativeEncoder motorBLEncoder = motorBL.GetEncoder();
  rev::SparkMaxRelativeEncoder motorBREncoder = motorBR.GetEncoder();

  int autonMode;

  // Xbox controller object contruct, does not contain correct port, pilot goes in 0 copilot goes in 1
  ModifiableController pilot = ModifiableController(0);
  ModifiableController copilot = ModifiableController(1);

  nt::NetworkTableInstance networkTableInstance = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> visionTab = networkTableInstance.GetTable("Shuffleboard")->GetSubTable("vision");


  // Declare doubles to store joystick values
  // Copilot joystick values, not currently using these values
 
  
  // Whether inputs to TankDrive() should be squared (increases sensitivity of inputs at low speed)

  //MADE IT INIT
  bool inputSentivityReduction;
  
  bool ebrake = true;
  bool arcadeDrive = true;
  double tankAssist = 0.08;

  // Deadzone values
  double deadzoneLimit = 0.05;

  const double driftInputSensitivity = 1.0;
  double defaultInputSensitivity = 0.4;

  double turningSensitivity = 0.6;



  // Create motor controller groups
  frc::MotorControllerGroup motorGroupLeft = frc::MotorControllerGroup(motorFL, motorBL);
  frc::MotorControllerGroup motorGroupRight = frc::MotorControllerGroup(motorFR, motorBR);

  // Pass motor controller groups to drivetrain object (also instantiate drivertrain object)
  frc::DifferentialDrive drivetrain = frc::DifferentialDrive(motorGroupLeft, motorGroupRight);
  
  // This is where we will put code for our motors and other sensors for the robot
  // The motors that we will be using for the drivetrain are NEO motors, so work can begin here once the electrical board is finished

  const double WheelRadiusInches = 3.0;

  bool firstCallToAuton = true;

  bool autonMovingMotor = false;

  int autonStep = 1;


  bool newAutonCall = true;

  double motorFLEncoderTarget;
  double motorFREncoderTarget;
  double motorBLEncoderTarget;
  double motorBREncoderTarget;
  
  bool invertRobot;

  double gearRatio =  20.0/3.0 * 3.0/2.0 * 30.0/29.0 * 30.0/29.5;
  double rotationAxisRadius = 13;

  bool autoAligning = false; 
};
