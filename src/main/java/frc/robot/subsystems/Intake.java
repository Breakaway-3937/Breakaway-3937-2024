// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final CANSparkMax frontIntakeMotor, backIntakeMotor;
  private final AnalogInput intake, staging, shooter;
  private final GenericEntry intakeSensor, stagingSensor, shooterSensor;

  /** Creates a new Intake. */
  public Intake() {
    frontIntakeMotor = new CANSparkMax(Constants.Intake.FRONT_INTAKE_MOTOR_ID, MotorType.kBrushless);
    backIntakeMotor = new CANSparkMax(Constants.Intake.BACK_INTAKE_MOTOR_ID, MotorType.kBrushless);
    configIntakeMotors();
    intake = new AnalogInput(Constants.Intake.INTAKE_SENSOR_ID);
    intake.resetAccumulator();
    staging = new AnalogInput(Constants.Intake.STAGING_SENSOR_ID);
    staging.resetAccumulator();
    shooter = new AnalogInput(Constants.Intake.SHOOTER_SENSOR_ID);
    shooter.resetAccumulator();
    intakeSensor = Shuffleboard.getTab("Intake").add("Intake Sensor", 0).withPosition(0, 0).getEntry();
    stagingSensor = Shuffleboard.getTab("Intake").add("Staging Sensor", 0).withPosition(1, 0).getEntry();
    shooterSensor = Shuffleboard.getTab("Intake").add("Shooter Sensor", 0).withPosition(2, 0).getEntry();
  }

  private void configIntakeMotors(){
    frontIntakeMotor.restoreFactoryDefaults();
    frontIntakeMotor.setIdleMode(IdleMode.kBrake);

    backIntakeMotor.restoreFactoryDefaults();
    backIntakeMotor.setIdleMode(IdleMode.kBrake);

    frontIntakeMotor.setSmartCurrentLimit(35);
    backIntakeMotor.setSmartCurrentLimit(35);

    backIntakeMotor.follow(frontIntakeMotor);
  }

  public void intake(){
    frontIntakeMotor.set(1);
  }

  public void spit(){
    frontIntakeMotor.set(-1);
  }

  public void stop(){
    frontIntakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeSensor.setDouble(intake.getValue());
    stagingSensor.setDouble(staging.getValue());
    shooterSensor.setDouble(shooter.getValue());
    Logger.getInstance().recordOutput("Intake", intake.getValue());
    Logger.getInstance().recordOutput("Staging", staging.getValue());
    Logger.getInstance().recordOutput("Shooter", shooter.getValue());
  }
}
