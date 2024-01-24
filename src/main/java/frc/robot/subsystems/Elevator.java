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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private final CANSparkMax elevatorMotor, followerElevatorMotor, babyWrist, babyShooter;
  private RelativeEncoder elevatorEncoder, babyWristEncoder;
  private SparkPIDController ePid, babyPid;
  private final GenericEntry elevatorEncoderEntry, brakeEntry, babyWristEntry;
  private final DoubleSolenoid brake;
  private boolean brakeBool;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new CANSparkMax(Constants.Elevator.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    followerElevatorMotor = new CANSparkMax(Constants.Elevator.FOLLOWER_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    configElevatorMotors();
    brake = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Shooter.FORWARD_CHANNEL, Constants.Shooter.REVERSE_CHANNEL);
    setBrakeOff();
    babyWrist = new CANSparkMax(Constants.Elevator.BABY_WRIST_ID, MotorType.kBrushless);
    babyShooter = new CANSparkMax(Constants.Elevator.BABY_SHOOTER_ID, MotorType.kBrushless);
    babyWristEntry = Shuffleboard.getTab("Elevator").add("Baby Wrist", getBabyWrist()).withPosition(2,0).getEntry();
    elevatorEncoderEntry = Shuffleboard.getTab("Elevator").add("Elevator", getElevator()).withPosition(0,0).getEntry();
    brakeEntry = Shuffleboard.getTab("Elevator").add("Brake", false).withPosition(1, 0).getEntry();
  }

  public void runBabyShooterForward(){
    babyShooter.set(1);
  }

  public void runBabyShooterReverse(){
    babyShooter.set(-1);
  }

  public void stopBabyShooter(){
    babyShooter.set(0);
  }

  public void setBabyWrist(double position){
    babyPid.setReference(position, ControlType.kSmartMotion);
  }

  public double getBabyWrist(){
    return babyWristEncoder.getPosition();
  }

  public void setElevator(double position){
    ePid.setReference(position, ControlType.kSmartMotion);
  }

  public double getElevator(){
    return elevatorEncoder.getPosition();
  }

  public void setBrakeOn(){
    brake.set(Value.kForward);
    brakeBool = true;
  }

  public void setBrakeOff(){
    brake.set(Value.kReverse);
    brakeBool = false;
  }

  public boolean getBrake(){
    return brakeBool;
  }

  private void configElevatorMotors(){
    elevatorMotor.restoreFactoryDefaults();
    followerElevatorMotor.restoreFactoryDefaults();
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    followerElevatorMotor.setIdleMode(IdleMode.kBrake);
    babyWrist.restoreFactoryDefaults();
    babyShooter.restoreFactoryDefaults();
    babyWrist.setIdleMode(IdleMode.kBrake);
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
    ePid.setSmartMotionMaxVelocity(0, 0); //FIXME
    ePid.setSmartMotionMaxAccel(0, 0); //FIXME
    ePid.setP(0); //FIXME
    ePid.setI(0);
    ePid.setD(0); //FIXME
    ePid.setFF(0); //FIXME
    babyPid.setOutputRange(-1, 1);
    babyPid.setSmartMotionMaxVelocity(0, 0); //FIXME
    babyPid.setSmartMotionMaxAccel(0, 0); //FIXME
    babyPid.setP(0); //FIXME
    babyPid.setI(0);
    babyPid.setD(0); //FIXME
    babyPid.setFF(0); //FIXME

    followerElevatorMotor.follow(elevatorMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorEncoderEntry.setDouble(getElevator());
    Logger.recordOutput("Elevator", getElevator());
    brakeEntry.setBoolean(getBrake());
    Logger.recordOutput("ElevatorBrake", getBrake());
    babyWristEntry.setDouble(getBabyWrist());
    Logger.recordOutput("Baby Wrist", getBabyWrist());
  }
}
