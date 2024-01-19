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

  private final CANSparkMax elevatorMotor, followerElevatorMotor;
  private RelativeEncoder elevatorEncoder;
  private SparkPIDController pid;
  private final GenericEntry elevatorEncoderEntry, brakeEntry;
  private final DoubleSolenoid brake;
  private boolean brakeBool;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new CANSparkMax(Constants.Elevator.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    followerElevatorMotor = new CANSparkMax(Constants.Elevator.FOLLOWER_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    configElevatorMotors();
    brake = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Shooter.FORWARD_CHANNEL, Constants.Shooter.REVERSE_CHANNEL);
    setBrakeOff();
    elevatorEncoderEntry = Shuffleboard.getTab("Elevator").add("Elevator", getElevator()).withPosition(0,0).getEntry();
    brakeEntry = Shuffleboard.getTab("Elevator").add("Brake", false).withPosition(1, 0).getEntry();
  }

  public void setElevator(double position){
    pid.setReference(position, ControlType.kSmartMotion);
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

    elevatorMotor.setSmartCurrentLimit(35);

    elevatorEncoder = elevatorMotor.getEncoder();
    pid = elevatorMotor.getPIDController();
    pid.setFeedbackDevice(elevatorEncoder);
    elevatorEncoder.setPosition(0);
    pid.setOutputRange(-1, 1);
    pid.setSmartMotionMaxVelocity(0, 0); //FIXME
    pid.setSmartMotionMaxAccel(0, 0); //FIXME
    pid.setP(0); //FIXME
    pid.setI(0);
    pid.setD(0); //FIXME
    pid.setFF(0); //FIXME

    followerElevatorMotor.follow(elevatorMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorEncoderEntry.setDouble(getElevator());
    Logger.recordOutput("Elevator", getElevator());
    brakeEntry.setBoolean(getBrake());
    Logger.recordOutput("ElevatorBrake", getBrake());
  }
}
