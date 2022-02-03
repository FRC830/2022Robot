// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
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
  void PlaceShuffleboardTiles();
  void GetTeleopShuffleBoardValues();
  void GetControllerInput();
  double Deadzone(double pilotStickY);
  double Avg(double val1, double val2);


  /*=============
  Pins & IDs
  =============*/




  // The line below is just an example, it does not contain the correct deviceID, may not contain
  // correct MotorType: Brushless
  rev::CANSparkMax motorFL = rev::CANSparkMax(1, rev::CANSparkMaxLowLevel::MotorType::kBrushless); 
  rev::CANSparkMax motorFR = rev::CANSparkMax(3, rev::CANSparkMaxLowLevel::MotorType::kBrushless); 
  rev::CANSparkMax motorBL = rev::CANSparkMax(2, rev::CANSparkMaxLowLevel::MotorType::kBrushless); 
  rev::CANSparkMax motorBR = rev::CANSparkMax(4, rev::CANSparkMaxLowLevel::MotorType::kBrushless); 

  // Xbox controller object contruct, does not contain correct port, pilot goes in 0 copilot goes in 1
  frc::XboxController pilot = frc::XboxController(0);
  frc::XboxController copilot = frc::XboxController(1);

  // Declare doubles to store joystick values
  // Copilot joystick values, not currently using these values
  double pilotLeftStickY;
  double pilotRightStickY;
  double pilotLeftStickX;
  double pilotRightStickX;
  double pilotLeftTriggerAxis; 
  double pilotRightTriggerAxis; 
  bool pilotLeftBumper;
  bool pilotRightBumper;
  bool pilotLeftBumperPressed;
  bool pilotRightBumperPressed;
  bool pilotLeftBumperReleased;
  bool pilotRightBumperReleased;
  bool pilotLeftStickButton;
  bool pilotRightStickButton;
  bool pilotLeftStickButtonPressed;
  bool pilotRightStickButtonPressed;
  bool pilotLeftStickButtonReleased;
  bool pilotRightStickButtonReleased;
  bool pilotAButton;
  bool pilotAButtonPressed;
  bool pilotAButtonReleased;
  bool pilotBButton;
  bool pilotBButtonPressed;
  bool pilotBButtonReleased;
  bool pilotXButton;
  bool pilotXButtonPressed;
  bool pilotXButtonReleased;
  bool pilotYButton;
  bool pilotYButtonPressed;
  bool pilotYButtonReleased;
  bool pilotBackButton;
  bool pilotBackButtonPressed;
  bool pilotBackButtonReleased;
  bool pilotStartButton;
  bool pilotStartButtonPressed;
  bool pilotStartButtonReleased;

  double copilotLeftStickY;
  double copilotRightStickY;
  double copilotLeftStickX;
  double copilotRightStickX;
  double copilotLeftTriggerAxis; 
  double copilotRightTriggerAxis;
  bool copilotLeftBumper;
  bool copilotRightBumper;
  bool copilotLeftBumperPressed;
  bool copilotRightBumperPressed;
  bool copilotLeftBumperReleased;
  bool copilotRightBumperReleased;
  bool copilotLeftStickButton;
  bool copilotRightStickButton;
  bool copilotLeftStickButtonPressed;
  bool copilotRightStickButtonPressed;
  bool copilotLeftStickButtonReleased;
  bool copilotRightStickButtonReleased;
  bool copilotAButton;
  bool copilotAButtonPressed;
  bool copilotAButtonReleased;
  bool copilotBButton;
  bool copilotBButtonPressed;
  bool copilotBButtonReleased;
  bool copilotXButton;
  bool copilotXButtonPressed;
  bool copilotXButtonReleased;
  bool copilotYButton;
  bool copilotYButtonPressed;
  bool copilotYButtonReleased;
  bool copilotBackButton;
  bool copilotBackButtonPressed;
  bool copilotBackButtonReleased;
  bool copilotStartButton;
  bool copilotStartButtonPressed;
  bool copilotStartButtonReleased;
  
  // Whether inputs to TankDrive() should be squared (increases sensitivity of inputs at low speed)

  //MADE IT INIT
  bool inputSentivityReduction;

  bool arcadeDrive = true;
  double tankAssist = 0.08;

  // Deadzone values
  double deadzoneLimit = 0.05;

  const double driftInputSensitivity = 1.0;
  double defaultinputSensitivity = 0.4;

  double turningSensitivity = 0.6;



  // Create motor controller groups
  frc::MotorControllerGroup motorGroupLeft = frc::MotorControllerGroup(motorFL, motorBL);
  frc::MotorControllerGroup motorGroupRight = frc::MotorControllerGroup(motorFR, motorBR);

  // Pass motor controller groups to drivetrain object (also instantiate drivertrain object)
  frc::DifferentialDrive drivetrain = frc::DifferentialDrive(motorGroupLeft, motorGroupRight);
  
  // This is where we will put code for our motors and other sensors for the robot
  // The motors that we will be using for the drivetrain are NEO motors, so work can begin here once the electrical board is finished
};
