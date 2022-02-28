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
  void HandleDrivetrain();
  void HandleShooter();
  void PlaceShuffleboardTiles();
  void GetTeleopShuffleBoardValues();
  void GetControllerInput();
  void HandleSolenoids();
  void HandleIntake();
  void HandleBallManagement();
  double Deadzone(double pilotStickY);
  double Avg(double val1, double val2);


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

  // Xbox controller object contruct, does not contain correct port, pilot goes in 0 copilot goes in 1
  ModifiableController pilot = ModifiableController(0);
  ModifiableController copilot = ModifiableController(1);

  // Create motor controller groups
  frc::MotorControllerGroup motorGroupLeft = frc::MotorControllerGroup(motorFL, motorBL);
  frc::MotorControllerGroup motorGroupRight = frc::MotorControllerGroup(motorFR, motorBR);

  // Pass motor controller groups to drivetrain object (also instantiate drivertrain object)
  frc::DifferentialDrive drivetrain = frc::DifferentialDrive(motorGroupLeft, motorGroupRight);
  
  /*
  #2 Intake & Ball Management
  */

  //Pnematic Initial Values
  frc::DoubleSolenoid doubleSolenoid{frc::PneumaticsModuleType::CTREPCM, 1, 2};  

  // Motors Needed to run the Intake (ids are arbritrary values we'll change later)
  ctre::phoenix::motorcontrol::can::VictorSPX intakeMotor{3};

  // Motor for Ball Management (ids are arbritrary values we'll change later)
  ctre::phoenix::motorcontrol::can::VictorSPX leftVictor{24}; //# is arbitrary put in device number later
  ctre::phoenix::motorcontrol::can::VictorSPX middleVictor{25}; //# is arbitrary put in device number later
  ctre::phoenix::motorcontrol::can::VictorSPX rightVictor{26}; //# is arbitrary put in device number later

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

  bool arcadeDrive = true;
  double tankAssist = 0.08;

  // Deadzone values
  double deadzoneLimit = 0.05;

  const double driftInputSensitivity = 1.0;
  double defaultInputSensitivity = 0.4;

  double turningSensitivity = 0.6;

  /*
  #2 Shooter
  */
  // (Arbritrary) Trigger values for Copilot Shooter
  float shooterMaximum = 0.5;
  double shooterOutput = 0;

  /*
  #3 Ball Management
  */

  float ballManageMaximum = 0.05;
  double ballManageOutput = 0;
 
};
