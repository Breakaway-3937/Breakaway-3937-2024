// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private final CANSparkMax elevatorMotor, followerElevatorMotor, babyWrist, babyShooter;
  private RelativeEncoder elevatorEncoder, babyWristEncoder;
  private SparkPIDController ePid, babyPid;
  private final GenericEntry elevatorEncoderEntry, babyWristEntry;
  private double ePosition, babyPosition;

  /** Creates a new Elevator. */
  public Elevator() {
    babyShooter = new CANSparkMax(Constants.Elevator.BABY_SHOOTER_ID, MotorType.kBrushless);
    elevatorMotor = new CANSparkMax(Constants.Elevator.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    followerElevatorMotor = new CANSparkMax(Constants.Elevator.FOLLOWER_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    babyWrist = new CANSparkMax(Constants.Elevator.BABY_WRIST_ID, MotorType.kBrushless);
    configMotors();
    babyWristEntry = Shuffleboard.getTab("Elevator").add("Baby Wrist", getBabyWrist()).withPosition(1,0).getEntry();
    elevatorEncoderEntry = Shuffleboard.getTab("Elevator").add("Elevator", getElevator()).withPosition(0,0).getEntry();
  }

  public void setElevatorFast(){
    ePid.setSmartMotionMaxVelocity(5500, 0);
    ePid.setSmartMotionMaxAccel(4000, 0);
  }

  public void setElevatorSlow(){
    ePid.setSmartMotionMaxVelocity(3000, 0);
    ePid.setSmartMotionMaxAccel(2500, 0);
  }

  public void setElevatorEvenSlower(){
    ePid.setSmartMotionMaxVelocity(1000, 0);
    ePid.setSmartMotionMaxAccel(500, 0);
  }

  public void setBabyShooterCoast(){
    babyShooter.setIdleMode(IdleMode.kCoast);
  }

  public void setBabyShooterBrake(){
    babyShooter.setIdleMode(IdleMode.kBrake);
  }

  public void runBabyShooterForward(){
    babyShooter.set(0.9);
  }

  public void runBabyShooterForwardSlowly(){
    babyShooter.set(0.5);
  }

  public void runBabyShooterReverse(){
    babyShooter.set(-0.9);
  }

  public void stopBabyShooter(){
    babyShooter.stopMotor();
  }

  public void setBabyWrist(double position){
    babyPid.setReference(position, ControlType.kSmartMotion);
    babyPosition = position;
  }

  public double getBabyWrist(){
    return babyWristEncoder.getPosition();
  }

  public void setElevator(double position){
    ePid.setReference(position, ControlType.kSmartMotion);
    ePosition = position;
  }

  public double getElevator(){
    return elevatorEncoder.getPosition();
  }

  public boolean isAtPosition(){
    if(getElevator() <= ePosition + 0.5 && getElevator() >= ePosition - 0.5 && getBabyWrist() <= babyPosition + 0.5 && getBabyWrist() >= babyPosition - 0.5){
      return true;
    }
    else{
      return false;
    }
  }

  private void configMotors(){
    elevatorMotor.restoreFactoryDefaults();
    followerElevatorMotor.restoreFactoryDefaults();
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    followerElevatorMotor.setIdleMode(IdleMode.kBrake);
    babyWrist.restoreFactoryDefaults();
    babyWrist.setIdleMode(IdleMode.kBrake);
    babyShooter.restoreFactoryDefaults();
    babyShooter.setIdleMode(IdleMode.kBrake);

    elevatorMotor.setSmartCurrentLimit(35);
    followerElevatorMotor.setSmartCurrentLimit(35);
    babyWrist.setSmartCurrentLimit(35);
    babyShooter.setSmartCurrentLimit(35);

    elevatorEncoder = elevatorMotor.getEncoder();
    babyWristEncoder = babyWrist.getEncoder();
    ePid = elevatorMotor.getPIDController();
    babyPid = babyWrist.getPIDController();
    ePid.setFeedbackDevice(elevatorEncoder);
    babyPid.setFeedbackDevice(babyWristEncoder);
    elevatorEncoder.setPosition(0);
    babyWristEncoder.setPosition(0);
    ePid.setOutputRange(-1, 1);
    ePid.setSmartMotionMaxVelocity(5500, 0);
    ePid.setSmartMotionMaxAccel(4000, 0);
    ePid.setP(0);
    ePid.setI(0);
    ePid.setD(0);
    ePid.setFF(0.0003);
    babyPid.setOutputRange(-1, 1);
    babyPid.setSmartMotionMaxVelocity(9000, 0);
    babyPid.setSmartMotionMaxAccel(8000, 0);
    babyPid.setP(0);
    babyPid.setI(0);
    babyPid.setD(0);
    babyPid.setFF(0.00015);

    followerElevatorMotor.follow(elevatorMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorEncoderEntry.setDouble(getElevator());
    Logger.recordOutput("Elevator", getElevator());
    babyWristEntry.setDouble(getBabyWrist());
    Logger.recordOutput("Baby Wrist", getBabyWrist());
  }
}
