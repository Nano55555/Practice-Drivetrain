// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class driveSubsystem extends SubsystemBase {
  /** Creates a new driveSubsystem. */
  private static final WPI_TalonFX leftFrontMotor= Constants.leftfrontmotor;
  private static final WPI_TalonFX rightFrontMotor= Constants.rightfrontmotor;
  private static final WPI_TalonFX leftBackMotor= Constants.leftbackmotor;
  private static final WPI_TalonFX rightBackMotor= Constants.rightbackmotor;

  private static final double In_To_M=.0254;
  private static final int Motor_Encoder_Codes_Per_Rev=2048;
  private static final double Diameter_Inches=5.0;
  private static final double Wheel_Diameter= Diameter_Inches * In_To_M;
  private static final double Wheel_Circumference= Wheel_Diameter * Math.PI;
  private static final double Gear_Ratio=12.75;
  private static final double Ticks_Per_Meter= ( Motor_Encoder_Codes_Per_Rev * Gear_Ratio)/(Wheel_Circumference);
  private static final double Meters_Per_Ticks= 1/Ticks_Per_Meter;

  public double left_speed_feedback;
  public double right_speed_feedback;

  public double left_speed_cmd;
  public double right_speed_cmd;


  /** Creates a new DriveSubsystem. */
  public driveSubsystem() {
    leftFrontMotor.set(ControlMode.Follower, leftBackMotor.getDeviceID());
    rightFrontMotor.set(ControlMode.Follower, rightBackMotor.getDeviceID()); 

    leftFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    leftBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

    leftFrontMotor.configNominalOutputForward(0, 10);
    leftFrontMotor.configNominalOutputReverse(0, 10);
    leftFrontMotor.configPeakOutputForward(1, 10);
    leftFrontMotor.configPeakOutputReverse(-1, 10);
    leftFrontMotor.configNeutralDeadband(0.001, 10);

    rightFrontMotor.configNominalOutputForward(0, 10);
    rightFrontMotor.configNominalOutputReverse(0, 10);
    rightFrontMotor.configPeakOutputForward(1, 10);
    rightFrontMotor.configPeakOutputReverse(-1, 10);
    rightFrontMotor.configNeutralDeadband(0.001, 10);

    leftBackMotor.configNominalOutputForward(0, 10);
    leftBackMotor.configNominalOutputReverse(0, 10);
    leftBackMotor.configPeakOutputForward(1, 10);
    leftBackMotor.configPeakOutputReverse(-1, 10);
    leftBackMotor.configNeutralDeadband(0.001, 10);

    rightBackMotor.configNominalOutputForward(0, 10);
    rightBackMotor.configNominalOutputReverse(0, 10);
    rightBackMotor.configPeakOutputForward(1, 10);
    rightBackMotor.configPeakOutputReverse(-1, 10);
    rightBackMotor.configNeutralDeadband(0.001, 10);

    // Sets how much error is allowed
    leftFrontMotor.configAllowableClosedloopError(0, 0, 10);
    leftBackMotor.configAllowableClosedloopError(0, 0, 10);
    rightFrontMotor.configAllowableClosedloopError(0, 0, 10);
    rightBackMotor.configAllowableClosedloopError(0, 0, 10);

    leftFrontMotor.setNeutralMode(NeutralMode.Coast); 
    rightFrontMotor.setNeutralMode(NeutralMode.Coast);
    leftBackMotor.setNeutralMode(NeutralMode.Coast);
    rightBackMotor.setNeutralMode(NeutralMode.Coast);
    
    leftFrontMotor.setInverted(true);
    rightFrontMotor.setInverted(false);
    leftBackMotor.setInverted(true);
    rightBackMotor.setInverted(false);

    leftBackMotor.setSensorPhase(true);
    leftFrontMotor.setSensorPhase(true);

    resetEncoders();
  }

  public void resetEncoders() {
    leftBackMotor.setSelectedSensorPosition(0);
    rightBackMotor.setSelectedSensorPosition(0);
    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
  }

  public double getRightBackEncoderPosition() {
    return rightBackMotor.getSelectedSensorPosition();
  }

  public double getLeftBackEncoderPosition() {
    return leftBackMotor.getSelectedSensorPosition();
  }

  public double distanceTravelledinTick(){
    return (getLeftBackEncoderPosition() + getRightBackEncoderPosition())/2;
  }
  public double getLeftBackEncoderPositionVelocityMetersPerSecond()
  {
    double leftVelocityMPS = (leftBackMotor.getSelectedSensorPosition()*10);
    leftVelocityMPS = leftVelocityMPS * Meters_Per_Ticks;
    return (leftVelocityMPS);
  }
  public double leftDistanceTravelledInMeters(){
    double left_dist=getRightBackEncoderPosition() * Meters_Per_Ticks;
    return left_dist;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void straightdrive(double leftspeed, double rightspeed){
    left_speed_feedback = getBackLeftEncoderVelocityMetersPerSecond();
    right_speed_feedback = getBackRightEncoderVelocityMetersPerSecond();

    left_speed_cmd = leftspeed;
    right_speed_cmd = rightspeed;

    // tick seconds / meter ms = ticks/meter * 1 second / 1000 ms * 100
    double driveSpeedPer100MS = (Ticks_Per_Meter * (1.0/1000.0) * 100.0); //tick second

    // leftSent = driveSpeedPer100MS*left_speed_cmd;
    // rightSent = driveSpeedPer100MS*right_speed_cmd;

    // // if the value being sent is lower than whatever value, then stop
    // if (leftSent < minDriveSpeed) {
    //   leftSent = 0;
    // } else {
    //   leftSent = leftSent;
    // }

    // if (rightSent < minDriveSpeed) {
    //   rightSent = 0;
    // } else {
    //   rightSent = rightSent;
    // }

    // passing in ticks/100 ms by multiplying (tick seconds / meter ms) * (meter/second) * 100
    // leftBackMotor.set(TalonFXControlMode.Velocity, leftSent); 
    // rightBackMotor.set(TalonFXControlMode.Velocity, rightSent);
    leftBackMotor.set(TalonFXControlMode.Velocity, driveSpeedPer100MS*left_speed_cmd); 
    rightBackMotor.set(TalonFXControlMode.Velocity, driveSpeedPer100MS*right_speed_cmd);

    /*leftFrontMotor.set(TalonFXControlMode.Velocity, driveSpeedPer100MS*left_speed_cmd); 
    rightFrontMotor.set(TalonFXControlMode.Velocity, driveSpeedPer100MS*right_speed_cmd);*/

    // actual speed command passed 
    /*left_speed_cmd = leftSpeed; // m/s
    right_speed_cmd = rightSpeed; // m/s
    // 1 meter/second * 0.1 second/100ms * 1 rev/(2pir meters) * gear ratio/1 rev * 2048 counts/rev
    int driveSpeedPer100MS = (int) ((0.1 * 10.71 * 2048) / (2 * Math.PI * 0.1524));
    leftBackMotor.set(TalonFXControlMode.Velocity, (int) (driveSpeedPer100MS*left_speed_cmd*5));
    rightBackMotor.set(TalonFXControlMode.Velocity, (int) (driveSpeedPer100MS*right_speed_cmd*5));
    */
  }
  public double distanceTravelledinTicks() {
    return (getBackLeftEncoderPosition() + getBackRightEncoderPosition()) / 2;
  }

  public double getBackLeftEncoderPosition() {
    return leftBackMotor.getSelectedSensorPosition();
  }

  public double getBackRightEncoderPosition() {
    return rightBackMotor.getSelectedSensorPosition();
  }

  public double getBackLeftEncoderVelocity() {
    return leftBackMotor.getSelectedSensorVelocity();
  }

  public double getBackRightEncoderVelocity() {
    return rightBackMotor.getSelectedSensorVelocity();
  }
  public double getBackLeftEncoderVelocityMetersPerSecond() {
    // getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double backLeftVelocityMPS = (leftBackMotor.getSelectedSensorVelocity() * 10);
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    backLeftVelocityMPS = backLeftVelocityMPS * Meters_Per_Ticks;
    return (backLeftVelocityMPS);
  }

  public double getBackRightEncoderVelocityMetersPerSecond() {
    // getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double backRightVelocityMPS = (rightBackMotor.getSelectedSensorVelocity() * 10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    // Need to have a negative for right velocity since the motors are reversed on
    // the opposite side
    backRightVelocityMPS = backRightVelocityMPS * Meters_Per_Ticks;
    return (backRightVelocityMPS);
  }
}


