#include <iostream>

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
  if (pilot.GetLeftBumper())
  {
    pilot.setSensitivityLevel(0.1);
  }
  else{
    pilot.setSensitivityLevel(defaultInputSensitivity + (((pilot.GetRightTriggerAxis("noS") < 0.9) ? pilot.GetRightTriggerAxis("noS") : 1)  * (1 - defaultInputSensitivity)));
  }


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

bool Robot::CalculateShot()
{
  int dist = visionTab -> GetNumber("distance", 180);
  

  frc::SmartDashboard::PutNumber("C++ thinks python dist is", dist);

  // index is the index of the
  // distance data point right above, -1 if its above everything
  int index=-2;
  for(int i =0; i< distances.size(); i++){
    if (dist < distances[i]){
      index=i-1;
      break;
    }
  }

  if(index >=0 && index < distances.size()-1){
    int indexbelow=index;
    int indexabove=index + 1;

    double distanceFromFloor = dist-distances[indexbelow];
    double ceil = distances[indexabove];
    double floor = distances[indexbelow];
    
    double proportionBetweenDistancePoints=(distanceFromFloor)/(distances[indexabove]-distances[indexbelow]);


    double targetRatio=((ratioMap[distances[indexabove]] - ratioMap[distances[indexbelow]])*proportionBetweenDistancePoints)+ratioMap[distances[indexbelow]];

    double targetSpeed=((speedMap[distances[indexabove]] - speedMap[distances[indexbelow]])*proportionBetweenDistancePoints)+speedMap[distances[indexbelow]];
    
    frc::SmartDashboard::PutNumber("proportionBetweenDistancePoints", proportionBetweenDistancePoints);
    frc::SmartDashboard::PutNumber("at 2", distances[1]);
    frc::SmartDashboard::PutNumber("distanceFromFloor", distanceFromFloor);
    frc::SmartDashboard::PutNumber("vision shooter speed", targetSpeed);
    frc::SmartDashboard::PutNumber("vision shooter ratio", targetRatio);
    frc::SmartDashboard::PutNumber("ceil", distances[indexabove]);
    frc::SmartDashboard::PutNumber("floor", distances[indexbelow]);
    
    correctSpeed = targetSpeed;
    correctRatio = targetRatio;

    return true;
  }
  else if (index==distances.size()-1)
  {
    return false;
  }
  else if(index==-1)
  {
    return false;
  } else 
  {
    return false;
  }
  
}

void Robot::HandleShooter(){
  //Apply Ryan's confusing Deadzone math:
  //The following line serves as a deadzone maximum ex: 0.7- (0.7-0.6)
  bool shotSuccess = CalculateShot();

  //std::tuple shotParams = ;

  //cout << std::get<0>(shotParams) << endl;

  // frc::SmartDashboard::PutNumber("vision shooter speed", correctSpeed);
  // frc::SmartDashboard::PutNumber("vision shooter ratio", correctRatio);

  //static shot (short)
  if (copilot.GetLeftTriggerAxis("noS") > 0.2)
  {
    leftFlywheelTalon.Set(TalonFXControlMode::Velocity, frc::SmartDashboard::GetNumber("Shooter Maximum", 4700));
    rightFlywheelTalon.Set(TalonFXControlMode::Follower, leftFlywheelTalon.GetDeviceID());
    backSpinTalon.Set(TalonFXControlMode::Velocity, frc::SmartDashboard::GetNumber("Shooter Maximum", 4700) * frc::SmartDashboard::GetNumber("ratio backspin to flywheel", 1));
    rightFlywheelTalon.SetInverted(true);
    backSpinTalon.SetInverted(true);
  }
  //long shot
  else if (copilot.GetYButton())
  {
    leftFlywheelTalon.Set(TalonFXControlMode::Velocity, 5000);
    rightFlywheelTalon.Set(TalonFXControlMode::Follower, leftFlywheelTalon.GetDeviceID());
    backSpinTalon.Set(TalonFXControlMode::Velocity, 5000 * 7.5);
    rightFlywheelTalon.SetInverted(true);
    backSpinTalon.SetInverted(true);
  }
  //eject
  else if (copilot.GetRightBumper())
  {
    leftFlywheelTalon.Set(TalonFXControlMode::Velocity, 3500);
    rightFlywheelTalon.Set(TalonFXControlMode::Follower, leftFlywheelTalon.GetDeviceID());
    backSpinTalon.Set(TalonFXControlMode::Velocity, 3000);
    rightFlywheelTalon.SetInverted(true);
    backSpinTalon.SetInverted(true);

  }
  //vision shot
  else if (copilot.GetRightTriggerAxis("noS") > 0.2 && shotSuccess){
  leftFlywheelTalon.Set(TalonFXControlMode::Velocity, correctSpeed);
  rightFlywheelTalon.Set(TalonFXControlMode::Follower, leftFlywheelTalon.GetDeviceID());
  backSpinTalon.Set(TalonFXControlMode::Velocity, correctSpeed * correctRatio);
  rightFlywheelTalon.SetInverted(true);
  backSpinTalon.SetInverted(true);

  }
  //no shot
  else
  {
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
  ballManageOutput = (copilot.GetXButton() || (copilot.GetAButton() && (leftFlywheelTalon.GetClosedLoopError() < 100))) ? frc::SmartDashboard::GetNumber("Ball Management Maximum", 0.5) : 0;
 

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
  intakeOutput = int(isIntaking) * 0.6;
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