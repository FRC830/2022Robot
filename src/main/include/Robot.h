// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ModifiableController.h>
#include <rev/CANSparkMax.h>
#include <frc/TimedRobot.h>
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
  void HandleClimber();
  //General util stuff
  void HandleShooter();
  void PlaceShuffleboardTiles();
  void GetTeleopShuffleBoardValues();
  void GetRobotShuffleoardValues();
  void GetControllerInput();
  void HandleIntake();
  void HandleBallManagement();
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
  void AccelerateFlywheelDuringAuton(int speed, double ratio);
  void RunBallManagement(double speed);
  void runIntake(double speed);
  //Auton Comb...
  void Taxi();
  void BackupAndShootAuton();
  void TestAuton();

 bool CalculateShot();


  std::map<int, double> ratioMap = {{89,	4},
                                     {111,	4.25},
                                     {134,	4.5},
                                     {156, 4.75},
                                     {172,	5.4},
                                     {188,	6},
                                     {207,	7}};
  std::map<int, double> speedMap = {{89,	4000},
                                     {111,	4000},
                                     {134,	4000},
                                     {156, 4500},
                                     {172,	4750},
                                     {188,	5250},
                                     {207,	5000}};

  std::array<int, 7> distances =
  {
    89,
    111,
    134,
    156,
    172,
    188,
    207
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

  // Mini Auton Section
  frc::SendableChooser<std::string> autonChooser;
  std::string stayAuton = "0) Do Nothing";
  std::string taxiAuton = "1) Taxi Auton";
  std::string stayLowAuton = "2) Do Nothing Low Goal Auton";
  std::string stayLowTaxiAuton = "3) Low goal shot + taxi";
  std::string oneBallAuton = "4) One Ball Auton";
  std::string twoBallLeftAuton = "5) Two Ball (Left) Auton";
  std::string twoBallMiddleAuton = "6) Two Ball (Middle) Auton";
  std::string twoBallRightAuton = "7) Two Ball (Right) Auton"; 
  std::string twoBallLineLeftAuton = "8) Two Ball Line Start (Left) Auton";
  std::string twoBallLineRightAuton = "9) Two Ball Line Start (Right) Auton";
  std::string twoBallLineMiddleAuton = "10) Two Ball Line Start (Middle) Auton";
  std::string mysteryMode = "11) Mystery Mode";

  // std::string threeBallAuton = "Three Ball Auton";

  /*
    Description of each Auton: 
    0 - stay still ✅
    1 - ?? stay still + low goal shot
    2 - back up and shoot left tarmac
    3 - back up and shoot right tarmac
    4 - back up intake and shoot left tarmac ✅
    5 - back up intake and shoot middle tarmac ✅
    6 - back up intake and shoot right tarmac ✅
    7 - line start back up and shoot left tarmac
    8 - line start back up and shoot right tarmac
    9 - ?? 3 ball auton
  */
  rev::CANSparkMax climber = rev::CANSparkMax(5, rev::CANSparkMaxLowLevel::MotorType::kBrushless); 
  rev::CANSparkMax climber2 = rev::CANSparkMax(6, rev::CANSparkMaxLowLevel::MotorType::kBrushless);

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
  
  // int auton_Wait = 0; 

  //this is a countdown to estimate the time unitl climber is fully extended it counts down 50 every second
  int climberCountdown = 200;


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

  double correctSpeed;
  double correctRatio;

  bool sneak = false;


};
