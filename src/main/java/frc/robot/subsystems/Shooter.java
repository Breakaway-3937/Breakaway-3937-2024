// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final TalonFX shooterMotor, shooterMotorOne;
  private TalonFXConfiguration shootMotorConfig = new TalonFXConfiguration();
  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
  private final GenericEntry shootEncoderEntry;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_ID);
    shooterMotorOne = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_ID); //FIXME
    configShootMotors();
    shootEncoderEntry = Shuffleboard.getTab("Shooter").add("ShootMotor", getShooterVelocity()).withPosition(0,0).getEntry();
  }

  public void runShooter(double speed){
    shooterMotor.setControl(request.withVelocity(speed));
  }
  
  public double getShooterVelocity(){
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  private void configShootMotors(){
    shooterMotor.getConfigurator().apply(new TalonFXConfiguration());
    shooterMotorOne.getConfigurator().apply(new TalonFXConfiguration());

    shootMotorConfig.Slot0.kP = 0; //FIXME
    shootMotorConfig.Slot0.kI = 0; //FIXME
    shootMotorConfig.Slot0.kD = 0; //FIXME

    shootMotorConfig.MotionMagic.MotionMagicAcceleration = 0; //FIXME
    shootMotorConfig.MotionMagic.MotionMagicJerk = 0; //FIXME

    shootMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    shooterMotor.getConfigurator().apply(shootMotorConfig);
    shooterMotorOne.getConfigurator().apply(shootMotorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shootEncoderEntry.setDouble(getShooterVelocity());
    Logger.recordOutput("Shooter", getShooterVelocity());
  }
}
