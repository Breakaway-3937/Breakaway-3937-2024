// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class Align extends Command {
    private final Swerve s_Swerve;
    private final Vision s_Vision;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private final BooleanSupplier robotCentricSup;
    private Rotation2d rotation = new Rotation2d();
    private boolean flag;

    private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagic)
      .withVelocityX(0.0)
      .withVelocityY(0.0);

    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagic)
      .withVelocityX(0.0)
      .withVelocityY(0.0);

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagic)
      .withVelocityX(0.0)
      .withVelocityY(0.0);

  /** Creates a new Align. */
  public Align(Swerve s_Swerve, Vision s_Vision, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    this.s_Vision = s_Vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve, s_Vision);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;

    swerveRequestFacing.HeadingController = new PhoenixPIDController(5, 0, 0);
    swerveRequestFacing.HeadingController.setTolerance(0.1);
    swerveRequestFacing.HeadingController.enableContinuousInput(-5, 5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flag = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RunElevator.amp && Robot.robotContainer.s_Intake.botFull()){
      rotation = Rotation2d.fromDegrees(90);
      s_Swerve.setControl(swerveRequestFacing.withVelocityX(translationSup.getAsDouble())
                                             .withVelocityY(strafeSup.getAsDouble())
                                             .withTargetDirection(rotation));
    }
    else if(!RunElevator.deadShooter || (RunElevator.amp && !Robot.robotContainer.s_Intake.botFull())){
      /* Drive */
      if(!Robot.robotContainer.s_Intake.botFull()){
        if(s_Vision.getNoteTargets()){
          if(robotCentricSup.getAsBoolean()){
            s_Swerve.setControl(robotCentric.withVelocityX(translationSup.getAsDouble())
                                            .withVelocityY(strafeSup.getAsDouble())
                                            .withRotationalRate(s_Vision.getNoteRotationSpeed()));
          }
          else{
            s_Swerve.setControl(fieldCentric.withVelocityX(translationSup.getAsDouble())
                                            .withVelocityY(strafeSup.getAsDouble())
                                            .withRotationalRate(s_Vision.getNoteRotationSpeed()));
          }
        }
        else{
          s_Swerve.setControl(fieldCentric.withVelocityX(translationSup.getAsDouble())
                                          .withVelocityY(strafeSup.getAsDouble())
                                          .withRotationalRate(rotationSup.getAsDouble()));
        }
      }
      else{
        if(s_Vision.getAprilTagRotationSpeed() != 0){
          s_Swerve.setControl(fieldCentric.withVelocityX(translationSup.getAsDouble())
                                          .withVelocityY(strafeSup.getAsDouble())
                                          .withRotationalRate(s_Vision.getAprilTagRotationSpeed()));
        }
        else{
          s_Swerve.setControl(swerveRequestFacing.withVelocityX(translationSup.getAsDouble())
                                                .withVelocityY(strafeSup.getAsDouble())
                                                .withTargetDirection(s_Vision.getOdometryRotationSpeed()));
        }
      }
    }

    if(Math.abs(s_Vision.getAprilTagYaw()) < 3 && !flag){
      Robot.robotContainer.s_LED.reset();
      Robot.robotContainer.s_LED.green();
      flag = true;
    }
    if(Math.abs(s_Vision.getAprilTagYaw()) >= 3){
      flag = false;
      Robot.robotContainer.s_LED.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.robotContainer.s_LED.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}