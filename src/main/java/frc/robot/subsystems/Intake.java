// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
