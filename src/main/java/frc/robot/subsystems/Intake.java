// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final GenericEntry intakeEncoderEntry;
  private RelativeEncoder intakeEncoder;

  private final CANSparkMax intakeMotor;
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless);
    intakeEncoderEntry = Shuffleboard.getTab("Intake").add("IntakeMotor", getIntakePosition()).withPosition(0,0).getEntry();
  }

  private void configIntakeMotor(){
    intakeMotor.restoreFactoryDefaults();
    intakeEncoder = intakeMotor.getEncoder();
    intakeEncoder.setPosition(0);
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void intake(){
    intakeMotor.set(1);
  }

  public void spit(){
    intakeMotor.set(-1);
  }

  public void stop(){
    intakeMotor.stopMotor();
  }

  public double getIntakePosition(){
    return intakeEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeEncoderEntry.setDouble(getIntakePosition());
    Logger.getInstance().recordOutput("Intake", getIntakePosition());
  }
}
