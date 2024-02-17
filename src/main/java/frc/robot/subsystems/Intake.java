// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {

  private final TalonFX frontIntakeMotor, backIntakeMotor;
  private final TalonFX loaderMotor;
  private final TalonSRX bags;
  private final TalonFXConfiguration loaderMotorConfig = new TalonFXConfiguration();
  private final AnalogInput intake, shooter, babyShooter;
  private final GenericEntry intakeSensor, shooterSensor, babyShooterSensor;
  private boolean flag, flag1, flag2, note;
  public boolean autoIntake;

  /** Creates a new Intake. */
  public Intake() {
    frontIntakeMotor = new TalonFX(Constants.Intake.FRONT_INTAKE_MOTOR_ID);
    backIntakeMotor = new TalonFX(Constants.Intake.BACK_INTAKE_MOTOR_ID);
    bags = new TalonSRX(0); //FIXME
    loaderMotor = new TalonFX(Constants.Intake.LOADER_MOTOR_ID);
    configMotors();
    intake = new AnalogInput(Constants.Intake.INTAKE_SENSOR_ID);
    shooter = new AnalogInput(Constants.Intake.SHOOTER_SENSOR_ID);
    babyShooter = new AnalogInput(Constants.Intake.BABY_SHOOTER_SENSOR_ID);
    intakeSensor = Shuffleboard.getTab("Intake").add("Intake Sensor", 0).withPosition(0, 0).getEntry();
    shooterSensor = Shuffleboard.getTab("Intake").add("Shooter Sensor", 0).withPosition(1, 0).getEntry();
    babyShooterSensor = Shuffleboard.getTab("Intake").add("Baby Shooter Sensor", 0).withPosition(2, 0).getEntry();
  }

  public TalonFX getLoaderMotor(){
    return loaderMotor;
  }

  private void configMotors(){

    bags.configFactoryDefault();
    frontIntakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    backIntakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    loaderMotor.getConfigurator().apply(new TalonFXConfiguration());
    loaderMotorConfig.CurrentLimits.SupplyCurrentLimit = 35;
    loaderMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    loaderMotorConfig.CurrentLimits.SupplyCurrentThreshold = 50;
    loaderMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    loaderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    loaderMotor.getConfigurator().apply(loaderMotorConfig);
    frontIntakeMotor.getConfigurator().apply(loaderMotorConfig);
    backIntakeMotor.getConfigurator().apply(loaderMotorConfig);
  }

  public void intake(){
    bags.set(TalonSRXControlMode.PercentOutput, 1);
    frontIntakeMotor.set(1);
    loaderMotor.setControl(new DutyCycleOut(1));
  }

  public void spit(){
    frontIntakeMotor.set(-1);
    loaderMotor.setControl(new DutyCycleOut(-1));
  }

  public void spitSlowly(){
    frontIntakeMotor.set(-0.5);
    loaderMotor.setControl(new DutyCycleOut(-0.5));
  }

  public void stop(){
    frontIntakeMotor.stopMotor();
    loaderMotor.stopMotor();
  }

  public boolean getIntakeSensor(){
    if(intake.getValue() > 4000){
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
    if(getIntakeSensor() || getShooterSensor() || getBabyShooterSensor()){
      note = true;
    }
    if(getShooterSensor()){
      flag = true;
    }
    if(getBabyShooterSensor()){
      flag1 = true;
    }
    if(getIntakeSensor()){
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
    if(frontIntakeMotor.get() < 0 && flag2 && !getIntakeSensor()){
      note = false;
      flag2 = false;
    }

    intakeSensor.setDouble(intake.getValue());
    shooterSensor.setDouble(shooter.getValue());
    babyShooterSensor.setDouble(babyShooter.getValue());
    Logger.recordOutput("Intake", intake.getValue());
    Logger.recordOutput("Shooter Sensor", shooter.getValue());
    Logger.recordOutput("Baby Shooter", babyShooter.getValue());
  }
}