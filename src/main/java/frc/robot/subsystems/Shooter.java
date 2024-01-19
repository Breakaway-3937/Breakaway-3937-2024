// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

public class Shooter extends SubsystemBase {

  private final TalonFX shooterMotor, followerShooterMotor;
  private final TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
  private final CANSparkMax wristMotor;
  private RelativeEncoder wristEncoder;
  private SparkPIDController pid;
  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
  private final Follower followerRequest = new Follower(Constants.Shooter.SHOOTER_MOTOR_ID, false);
  private final GenericEntry shooterEncoderEntry, wristEncoderEntry;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_ID);
    followerShooterMotor = new TalonFX(Constants.Shooter.FOLLOWER_SHOOTER_MOTOR_ID);
    wristMotor = new CANSparkMax(Constants.Shooter.WRIST_MOTOR_ID, MotorType.kBrushless);
    configShooterMotors();
    configWristMotor();
    shooterEncoderEntry = Shuffleboard.getTab("Shooter").add("Shooter", getShooterVelocity()).withPosition(0,0).getEntry();
    wristEncoderEntry = Shuffleboard.getTab("Shooter").add("Wrist", getWrist()).withPosition(1, 0).getEntry();
  }

  public void runShooter(double speed){
    shooterMotor.setControl(request.withVelocity(speed));
  }
  
  public double getShooterVelocity(){
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  public void setWrist(double position){
    pid.setReference(position, ControlType.kSmartMotion);
  }

  public double getWrist(){
    return wristEncoder.getPosition();
  }

  private void configShooterMotors(){
    shooterMotor.getConfigurator().apply(new TalonFXConfiguration());
    followerShooterMotor.getConfigurator().apply(new TalonFXConfiguration());

    shooterMotorConfig.Slot0.kP = 0; //FIXME
    shooterMotorConfig.Slot0.kI = 0;
    shooterMotorConfig.Slot0.kD = 0; //FIXME

    shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 35;
    shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterMotorConfig.CurrentLimits.SupplyCurrentThreshold = 50;
    shooterMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

    shooterMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0; //FIXME
    shooterMotorConfig.MotionMagic.MotionMagicAcceleration = 0; //FIXME

    shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    shooterMotor.getConfigurator().apply(shooterMotorConfig);
    followerShooterMotor.getConfigurator().apply(shooterMotorConfig);

    followerShooterMotor.setControl(followerRequest);
  }

  private void configWristMotor(){
    wristMotor.restoreFactoryDefaults();
    wristMotor.setIdleMode(IdleMode.kBrake);

    wristMotor.setSmartCurrentLimit(35);

    wristEncoder = wristMotor.getEncoder();
    pid = wristMotor.getPIDController();
    pid.setFeedbackDevice(wristEncoder);
    wristEncoder.setPosition(0);
    pid.setOutputRange(-1, 1);
    pid.setSmartMotionMaxVelocity(0, 0); //FIXME
    pid.setSmartMotionMaxAccel(0, 0); //FIXME
    pid.setP(0); //FIXME
    pid.setI(0);
    pid.setD(0); //FIXME
    pid.setFF(0); //FIXME
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterEncoderEntry.setDouble(getShooterVelocity());
    Logger.recordOutput("Shooter", getShooterVelocity());
    wristEncoderEntry.setDouble(getWrist());
    Logger.recordOutput("Wrist", getWrist());
  }
}
