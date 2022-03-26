#include <iostream>

void Robot::runIntake(double speed)
{
  intakeOutput = speed * intakeMaximum;
  intakeMotor.Set(VictorSPXControlMode::PercentOutput, -0.9);
  frc::SmartDashboard::PutNumber("Intake Output", 0.9);

  leftSolenoid.Set(true);
  rightSolenoid.Set(true);
  frc::SmartDashboard::PutBoolean("Intake Extended", true);

  leftVictor.Set(VictorSPXControlMode::PercentOutput, speed);
  rightVictor.SetInverted(true);
  rightVictor.Set(VictorSPXControlMode::Follower, leftVictor.GetDeviceID());
  
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
  autonStep++;
}

void Robot::LinearMove(double distance, double motorSpeed)
{
  assert (motorSpeed > 0);
  assert (distance != 0);

  double encoderDistance = InchesToEncoderTicks(distance);
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

  //std::cout << "FR target: " << std::to_string (motorFREncoderTarget) << " FR Position: " << //::to_string (motorFREncoder.GetPosition()) << std::endl;
  //std::cout << "FL target: " << std::to_string (motorFLEncoderTarget) << " FL Position: " << s//td::to_string (motorFLEncoder.GetPosition()) << std::endl;
  //std::cout << "BL target: " << std::to_string (motorBLEncoderTarget) << " BL Position: " << std::to_string (motorBLEncoder.GetPosition()) << std::endl;
  //std::cout << "BR target: " << std::to_string (motorBREncoderTarget) << " BR Position: " << std::to_string (motorBREncoder.GetPosition()) << std::endl;

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