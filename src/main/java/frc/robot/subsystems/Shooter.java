// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final TalonFX shootMotor;
  private TalonFXConfiguration shootMotorConfig = new TalonFXConfiguration();
  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
  private final GenericEntry shootEncoderEntry;

  /** Creates a new Shooter. */
  public Shooter() {
    shootMotor = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_ID);
    shootMotor.getConfigurator().apply(new TalonFXConfiguration());
    shootMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shootMotor.getConfigurator().apply(shootMotorConfig);
    shootEncoderEntry = Shuffleboard.getTab("Shooter").add("ShootMotor", getShooterPosition()).withPosition(0,0).getEntry();
  }

  public void runShooter(){
    shootMotor.set
  }

  
  public double getShooterPosition(){
    return shootMotor.getRotorPosition().getValue();
  }

  private void configShootMotor(){
    shootMotor.getConfigurator().apply(new TalonFXConfiguration());

    shootMotorConfig.slot0Configs.kP = 0; //FIXME
    shootMotorConfig.slot0Configs.kI = 0; //FIXME
    shootMotorConfig.slot0Configs.kD = 0; //FIXME

    shootMotorConfig.motionMagicConfig.MotionMagicAcceleration = 0; //FIXME
    motionMagicConfigs.MotionMagicJerk = 4000;

    shootMotor.getConfigurator().apply(shootMotorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shootEncoderEntry.setDouble(getShooterPosition());
    Logger.getInstance().recordOutput("Shooter", getShooterPosition());
  }
}
