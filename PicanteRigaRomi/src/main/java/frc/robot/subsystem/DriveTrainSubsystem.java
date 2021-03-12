// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

//import com.fasterxml.jackson.databind.ser.std.StdArraySerializers.DoubleArraySerializer;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.controller.*;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsytem. */
  private final Spark leftMotor, rightMotor;
  private final Encoder leftEncoder, rightEncoder;
  private final RomiGyro gyro;
  private final PIDController pid;
  
  public DriveTrainSubsystem() {
    leftMotor = new Spark(Constants.DrivetrainConstants.LEFT_MOTOR_PORT);
    rightMotor = new Spark(Constants.DrivetrainConstants.RIGHT_MOTOR_PORT);
    leftEncoder = new Encoder(Constants.DrivetrainConstants.LEFT_ENCODER_A, Constants.DrivetrainConstants.LEFT_ENCODER_B);
    rightEncoder = new Encoder(Constants.DrivetrainConstants.RIGHT_ENCODER_A, Constants.DrivetrainConstants.RIGHT_ENCODER_B);
    leftEncoder.setDistancePerPulse(Constants.DrivetrainConstants.INCHES_PER_PULSE);
    rightEncoder.setDistancePerPulse(Constants.DrivetrainConstants.INCHES_PER_PULSE);
    rightMotor.setInverted(true);
    gyro = new RomiGyro();
    pid = new PIDController(0.15, 0, 0);
  }

  public void drive(double leftSpeed, double rightSpeed){
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }

  public void drivePID(double setpoint){
    leftMotor.set(pid.calculate(leftEncoder.getDistance(), setpoint));
    rightMotor.set(pid.calculate(rightEncoder.getDistance(), setpoint));
  }
  public void driveStraight(){
      drive(0.5, 0.5);   
  }

  public void turn(boolean isRight){
    double coefficient = isRight ? -1 : 1;
    drive(-coefficient*0.5, coefficient*0.5);
  }

  public double getOffset(){
    return pid.getPositionError();
  }

  public void resetGyro(){
    gyro.reset();
  }

  public double getAngle(){
    return gyro.getAngle();
  }

  public void resetEncoders(){
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getLeftDistance(){
    return leftEncoder.getDistance();
  }

  public double getRightDistance(){
    return rightEncoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
