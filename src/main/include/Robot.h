// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ModifiableController.h>
#include <rev/CANSparkMax.h>
#include "ctre/Phoenix.h"
#include <string>
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedController.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <cmath> 
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>

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
  void HandleShooter();
  void HandleIntake();
  void HandleBallManagement();
  //General util stuff
  double Deadzone(double pilotStickY);
  double Avg(double val1, double val2);
  double EncoderTicksToInches(double ticks, double TicksPerRev);
  double InchesToEncoderTicks(double inches, double TicksPerRev);
  double EncoderTicksToInches(double ticks);
  double InchesToEncoderTicks(double inches);
  double DegreesToInches(double degrees);
  // smartdash board stuff
  void PlaceShuffleboardTiles();
  void GetTeleopShuffleBoardValues();
  void GetRobotShuffleoardValues();
  void GetControllerInput();

  //Auton functions... for auton...
  void LinearMove(double distance, double motorSpeed);
  void CenterPointTurn(double degrees, double motorSpeed);
  bool AimRobotAtHub(double moterSpeed);
  void CompoundMove(double distance, double degrees, double motorSpeed);
  void AccelerateFlywheelDuringAuton(int speed, double ratio);
  void RunBallManagement(double speed);
  void RunIntake(double speed);
  //Auton Comb...
  void Taxi();
  void TwoBallAuton();
  void TestAuton();


  std::map<int, double> ratioMap = {{60, 0.8},
                                     {90, 0.6},
                                     {180, 0.6},
                                     {270, 0.6}};
  std::map<int, double> speedMap = {{60, 8750},
                                     {90, 11500},
                                     {180, 12500},
                                     {270, 14800}};

  std::array<int, 4> distances =
  {
    60,
    90,
    180,
    270
  };

  /*=============
  Pins & IDs
  =============*/

  /*
  #1 Drivetrain
  */
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
 
  // Create motor controller groups
  frc::MotorControllerGroup motorGroupLeft = frc::MotorControllerGroup(motorFL, motorBL);
  frc::MotorControllerGroup motorGroupRight = frc::MotorControllerGroup(motorFR, motorBR);

  // Pass motor controller groups to drivetrain object (also instantiate drivertrain object)
  frc::DifferentialDrive drivetrain = frc::DifferentialDrive(motorGroupLeft, motorGroupRight);
  
  /*
  #2 Intake & Ball Management
  */

  //Pnematic Initial Values
  frc::Solenoid leftSolenoid{frc::PneumaticsModuleType::CTREPCM, 0};
  frc::Solenoid rightSolenoid{frc::PneumaticsModuleType::CTREPCM, 1};

  // Motors Needed to run the Intake (ids are arbritrary values we'll change later)
  ctre::phoenix::motorcontrol::can::VictorSPX intakeMotor{3};

  // Motor for Ball Management (ids are arbritrary values we'll change later)
  ctre::phoenix::motorcontrol::can::VictorSPX leftVictor{24}; //ids for Vitors are correct
  ctre::phoenix::motorcontrol::can::VictorSPX middleVictor{25}; 
  ctre::phoenix::motorcontrol::can::VictorSPX rightVictor{26}; 

  /*
  #3 Shooter
  */
  // Shooter Motor IDs and ball managment
  ctre::phoenix::motorcontrol::can::TalonFX leftFlywheelTalon {4};
  ctre::phoenix::motorcontrol::can::TalonFX rightFlywheelTalon {9}; //Set as inverted and following leftFlywheelTalon
  ctre::phoenix::motorcontrol::can::TalonFX backSpinTalon {5}; //Set as inverted and following leftFlywheelTalon
  
    

  /*=============
  Constant Values
  =============*/

  /*
  #1 Drivetrain
  */

  // Declare doubles to store joystick values
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





  //THIS is in the wrong spot
  const int TIMERLENGTH = 25;
  int shootStablizer=0;

  /*
  #2 Shooter
  */
  // (Arbritrary) Trigger values for Copilot Shooter
  float shooterMaximum = 0.5;
  float shooterHANGER = 0.5; 
  double shooterOutput = 0;

  bool longSHOTHANGER = false; 

  /*
  #3 Ball Management
  */

  float ballManageMaximum = 0.05;
  double ballManageOutput = 0;

  
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
  /*
  #4 Intake
  */

  bool intakeExtended = false;
  float intakeMaximum = 1.0;
  double intakeOutput = 0;


};
