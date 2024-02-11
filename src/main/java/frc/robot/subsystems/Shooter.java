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

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final TalonFX shooterMotor, followerShooterMotor;
  private final TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
  private final CANSparkMax wristMotor, wristFollowerMotor;
  private RelativeEncoder wristEncoder;
  private SparkPIDController pid;
  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0).withSlot(0);
  private final Follower followerRequest = new Follower(Constants.Shooter.SHOOTER_MOTOR_ID, true);
  private final GenericEntry shooterEncoderEntry, wristEncoderEntry;
  private double speed, position = 0;
  private boolean subwoofer, podium;
  private final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap wristMap = new InterpolatingDoubleTreeMap();

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_ID);
    followerShooterMotor = new TalonFX(Constants.Shooter.FOLLOWER_SHOOTER_MOTOR_ID);
    wristMotor = new CANSparkMax(Constants.Shooter.WRIST_MOTOR_ID, MotorType.kBrushless);
    wristFollowerMotor = new CANSparkMax(Constants.Shooter.WRIST_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
    configShooterMotors();
    configWristMotor();
    shooterEncoderEntry = Shuffleboard.getTab("Shooter").add("Shooter", getShooterVelocity()).withPosition(0,0).getEntry();
    wristEncoderEntry = Shuffleboard.getTab("Shooter").add("Wrist", getWrist()).withPosition(1, 0).getEntry();
  }

  public Pair<TalonFX, TalonFX> getShooterMotors(){
    return new Pair<TalonFX, TalonFX>(shooterMotor, followerShooterMotor);
  }

  public void setSubShooting(){
    subwoofer = true;
    podium = false;
  }

  public void setPodiumShooting(){
    subwoofer = false;
    podium = true;
  }

  public void setAutoShooting(){
    subwoofer = false;
    podium = false;
  }

  public void runShooter(){
    shooterMotor.setControl(request.withVelocity(speed));
  }

  public void setShooter(double speed){
    shooterMotor.setControl(request.withVelocity(speed));
  }

  public void stopShooter(){
    shooterMotor.stopMotor();
  }

  public boolean atSpeed(){
    if(speed <= getShooterVelocity() + 1.67 && speed >= getShooterVelocity() - 1.67){
      return true;
    }
    else{
      return false;
    }
  }
  
  public double getShooterVelocity(){
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  public void setWristShooting(){
    if(podium){
      speed = 0; //FIXME setpoint
      position = 0;
    }
    else if(subwoofer){
      speed = 3000.0 / 60.0;
      position = 17.5;
    }
    /*else{
      speed = shooterMap.get(0.0/*Robot.robotContainer.s_Vision.getDistance()*///);
      //position = //wristMap.get(0.0/*Robot.robotContainer.s_Vision.getDistance()*///);
    //}
    pid.setReference(position, ControlType.kSmartMotion);
  }

  public void setWrist(double position){
    this.position = position;
    pid.setReference(position, ControlType.kSmartMotion);
  }

  public double getWrist(){
    return wristEncoder.getPosition();
  }

  public boolean isSafe(){
    if(getWrist() > 12.8 && getWrist() < 13.2){
      return true;
    }
    else{
      return false;
    }
  }

  public boolean isAtPosition(){
    if(getWrist() > position - 0.2 && getWrist() < position + 0.2){
      return true;
    }
    else{
      return false;
    }
  }

  public double getSpeed(){
    return speed;
  }

  public void setSpeedToZero(){
    speed = 0;
  }

  private void configShooterMotors(){
    shooterMotor.getConfigurator().apply(new TalonFXConfiguration());
    followerShooterMotor.getConfigurator().apply(new TalonFXConfiguration());

    shooterMotorConfig.Slot0.kS = 0.4; // Add 0.25 V output to overcome static friction
    shooterMotorConfig.Slot0.kV = 0.13; // A velocity target of 1 rps results in 0.12 V output
    shooterMotorConfig.Slot0.kA = 0.1; // An acceleration of 1 rps/s requires 0.01 V output
    shooterMotorConfig.Slot0.kP = 0.2; // An error of 1 rps results in 0.11 V output
    shooterMotorConfig.Slot0.kI = 0; // no output for integrated error
    shooterMotorConfig.Slot0.kD = 0; // no output for error derivative

    shooterMotorConfig.MotionMagic.MotionMagicAcceleration = 1200;
    shooterMotorConfig.MotionMagic.MotionMagicJerk = 4000;

    shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    shooterMotor.getConfigurator().apply(shooterMotorConfig);
    followerShooterMotor.getConfigurator().apply(shooterMotorConfig);

    followerShooterMotor.setControl(followerRequest);
  }

  private void configWristMotor(){
    wristMotor.restoreFactoryDefaults();
    wristFollowerMotor.restoreFactoryDefaults();
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristFollowerMotor.setIdleMode(IdleMode.kBrake);
    wristFollowerMotor.follow(wristMotor);

    wristMotor.setInverted(true);

    wristMotor.setSmartCurrentLimit(35);
    wristFollowerMotor.setSmartCurrentLimit(35);

    wristEncoder = wristMotor.getEncoder();
    pid = wristMotor.getPIDController();
    pid.setFeedbackDevice(wristEncoder);
    wristEncoder.setPosition(0);
    pid.setOutputRange(-1, 1);
    pid.setSmartMotionMaxVelocity(4250, 0);
    pid.setSmartMotionMaxAccel(4000, 0);
    pid.setP(0);
    pid.setI(0);
    pid.setD(0);
    pid.setFF(0.0002);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterEncoderEntry.setDouble(getShooterVelocity());
    Logger.recordOutput("Shooter", getShooterVelocity());
    wristEncoderEntry.setDouble(getWrist());
    Logger.recordOutput("Wrist", getWrist());
    /*SmartDashboard.putNumber("Shooter Speed", getSpeed());
    SmartDashboard.putNumber("Shoulder Posision", getWrist());
    speed = SmartDashboard.getNumber("Speed Input", 0);
    position = SmartDashboard.getNumber("Position of Wrist", 0);*/
  }
}
