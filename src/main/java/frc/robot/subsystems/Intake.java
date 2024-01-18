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

  /** Creates a new Intake. */
  public Intake() {
    frontIntakeMotor = new CANSparkMax(Constants.Intake.FRONT_INTAKE_MOTOR_ID, MotorType.kBrushless);
    backIntakeMotor = new CANSparkMax(Constants.Intake.BACK_INTAKE_MOTOR_ID, MotorType.kBrushless);
    configIntakeMotors();
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
  }
}
