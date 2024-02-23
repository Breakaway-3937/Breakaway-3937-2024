// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {

  private final TalonFX leadIntakeMotor, followerIntakeMotor, loaderMotor;
  private final TalonSRX bags;
  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  private final TalonSRXConfiguration bagsConfig = new TalonSRXConfiguration();
  private final AnalogInput popTart, shooter, babyShooter;
  private final GenericEntry popTartSensor, shooterSensor, babyShooterSensor;
  private final Follower followerRequest = new Follower(Constants.Intake.LEAD_INTAKE_MOTOR_ID, true);
  private boolean flag, flag1, flag2, note;
  public boolean autoIntake;

  /** Creates a new Intake. */
  public Intake() {
    leadIntakeMotor = new TalonFX(Constants.Intake.LEAD_INTAKE_MOTOR_ID);
    followerIntakeMotor = new TalonFX(Constants.Intake.FOLLOWER_INTAKE_MOTOR_ID);
    bags = new TalonSRX(Constants.Intake.BAGS_MOTOR_ID);
    loaderMotor = new TalonFX(Constants.Intake.LOADER_MOTOR_ID);
    configMotors();
    popTart = new AnalogInput(Constants.Intake.POP_TART_SENSOR_ID);
    shooter = new AnalogInput(Constants.Intake.SHOOTER_SENSOR_ID);
    babyShooter = new AnalogInput(Constants.Intake.BABY_SHOOTER_SENSOR_ID);
    popTartSensor = Shuffleboard.getTab("Intake").add("Pop-Tart Sensor", 0).withPosition(0, 0).getEntry();
    shooterSensor = Shuffleboard.getTab("Intake").add("Shooter Sensor", 0).withPosition(1, 0).getEntry();
    babyShooterSensor = Shuffleboard.getTab("Intake").add("Baby Shooter Sensor", 0).withPosition(2, 0).getEntry();
  }

  public Pair<TalonFX, TalonFX> getIntakeMotors(){
    return new Pair<TalonFX, TalonFX>(leadIntakeMotor, followerIntakeMotor);
  }

  public TalonFX getLoaderMotor(){
    return loaderMotor;
  }

  private void configMotors(){
    bags.configFactoryDefault();
    bags.setInverted(true);
    bagsConfig.peakCurrentDuration = 100;
    bagsConfig.peakCurrentLimit = 40;
    bagsConfig.continuousCurrentLimit = 25;
    bags.configAllSettings(bagsConfig);
    leadIntakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    followerIntakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    loaderMotor.getConfigurator().apply(new TalonFXConfiguration());
    motorConfig.CurrentLimits.SupplyCurrentLimit = 35;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = 50;
    motorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Audio.AllowMusicDurDisable = true;
    followerIntakeMotor.getConfigurator().apply(motorConfig);
    leadIntakeMotor.getConfigurator().apply(motorConfig);
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    loaderMotor.getConfigurator().apply(motorConfig);
    followerIntakeMotor.setControl(followerRequest);
  }

  public void intake(){
    leadIntakeMotor.setControl(new DutyCycleOut(0.9).withEnableFOC(true));
    loaderMotor.setControl(new DutyCycleOut(0.9).withEnableFOC(true));
    bags.set(TalonSRXControlMode.PercentOutput, 0.9);
  }

  public void intakeSlowly(){
    leadIntakeMotor.setControl(new DutyCycleOut(0.25).withEnableFOC(true));
    loaderMotor.setControl(new DutyCycleOut(0.25).withEnableFOC(true));
    bags.set(TalonSRXControlMode.PercentOutput, 0.25);
  }

  public void spit(){
    leadIntakeMotor.setControl(new DutyCycleOut(-0.9).withEnableFOC(true));
    loaderMotor.setControl(new DutyCycleOut(-0.9).withEnableFOC(true));
    bags.set(TalonSRXControlMode.PercentOutput, -0.9);
  }

  public void spitSlowly(){
    leadIntakeMotor.setControl(new DutyCycleOut(-0.5).withEnableFOC(true));
    loaderMotor.setControl(new DutyCycleOut(-0.5).withEnableFOC(true));
    bags.set(TalonSRXControlMode.PercentOutput, -0.5);
  }

  public void stop(){
    leadIntakeMotor.stopMotor();
    loaderMotor.stopMotor();
    bags.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public boolean getPopTartSensor(){
    if(popTart.getValue() > 4000){
      return true;
    }
    else{
      return false;
    }
  }

  public boolean getShooterSensor(){
    if(shooter.getValue() > 4000){
      return true;
    }
    else{
      return false;
    }
  }

  public boolean getBabyShooterSensor(){
    if(babyShooter.getValue() > 4000){
      return true;
    }
    else{
      return false;
    }
  }

  public boolean botFull(){
    return note;
  }

  public boolean autoIntake(){
    return autoIntake;
  }

  public void setAutoIntake(boolean autoIntake){
    this.autoIntake = autoIntake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(getPopTartSensor() || getShooterSensor() || getBabyShooterSensor()){
      note = true;
    }
    if(getShooterSensor()){
      flag = true;
    }
    if(getBabyShooterSensor()){
      flag1 = true;
    }
    if(getPopTartSensor()){
      flag2 = true;
    }
    if(Robot.robotContainer.s_Shooter.getSpeed() > 0 && loaderMotor.get() > 0 && flag && !getShooterSensor()){
      note = false;
      flag = false;
    }
    if(flag1 && !getBabyShooterSensor()){
      note = false;
      flag1 = false;
    }
    if(leadIntakeMotor.get() < 0 && flag2 && !getPopTartSensor()){
      note = false;
      flag2 = false;
    }

    popTartSensor.setDouble(popTart.getValue());
    shooterSensor.setDouble(shooter.getValue());
    babyShooterSensor.setDouble(babyShooter.getValue());
    Logger.recordOutput("Pop-Tart", popTart.getValue());
    Logger.recordOutput("Shooter Sensor", shooter.getValue());
    Logger.recordOutput("Baby Shooter", babyShooter.getValue());
  }
}