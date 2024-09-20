// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;


import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class Align extends Command {
    private final Swerve drivetrain;
    private final Vision vision;
    private final Joystick joystick;

    private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagic)
    .withVelocityX(0.0)
    .withVelocityY(0.0);

  /** Creates a new Align. */
  public Align(Swerve drivetrain, Vision vision, Joystick joystick) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.joystick = joystick;
    swerveRequestFacing.HeadingController = new PhoenixPIDController(5, 0, 0);
    swerveRequestFacing.HeadingController.setTolerance(0.1);
    swerveRequestFacing.HeadingController.enableContinuousInput(-5, 5);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setControl(swerveRequestFacing.withVelocityX(-joystick.getRawAxis(joystick.getPort()) * 5)
                                             .withVelocityY(-joystick.getRawAxis(joystick.getPort()) * 5)
                                             .withTargetDirection(Rotation2d.fromDegrees(vision.getAngleToAprilTag())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}