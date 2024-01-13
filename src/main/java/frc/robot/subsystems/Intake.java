// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX intake;
  private final DutyCycleOut move = new DutyCycleOut(0).withEnableFOC(true);
  /** Creates a new Intake. */
  public Intake() {
    intake = new TalonFX(6);

  }

  public void intake(){
    intake.setControl(move.withOutput(1));
  }

  public void spit(){
    intake.setControl(move.withOutput(-1));
  }

  public void stop(){
    intake.setControl(move.withOutput(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}