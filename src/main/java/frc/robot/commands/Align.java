// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
  private final double pValue = 8.0 / 42.0;
  private boolean flag, scheduled;
  private double translationVal, strafeVal, rotationVal;
  private Pose2d trapPose = new Pose2d();
  private final PathConstraints constraints = new PathConstraints(Constants.Swerve.MAX_SPEED, 4.5, 12.0428, 12.0428);
  private Command command;

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
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flag = false;
    scheduled = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RunElevator.amp){
      translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      if(!Robot.getRedAlliance()){
        rotationVal = pValue * (90 - (s_Swerve.getGyroYaw().getDegrees() + 3600000) % 360);
      }
      else{
        rotationVal = pValue * (270 - (s_Swerve.getGyroYaw().getDegrees() + 3600000) % 360);
      }
      s_Swerve.drive(
          new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
          rotationVal, 
          !robotCentricSup.getAsBoolean(),
          true
      );
    }
    else if(RunElevator.climbing){
      if(!Robot.getRedAlliance()){
        if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 3600000) % 360 > 90 || (Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 3600000) % 360 < 270){
          trapPose = new Pose2d(new Translation2d(Constants.Vision.TRAP_CENTER_TARGET_X_BLUE, Constants.Vision.TRAP_CENTER_TARGET_Y_BLUE), Rotation2d.fromDegrees(0));
        }
        else if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 3600000) % 360 > 270){
          trapPose = new Pose2d(new Translation2d(Constants.Vision.TRAP_LEFT_TARGET_X_BLUE, Constants.Vision.TRAP_LEFT_TARGET_Y_BLUE), Rotation2d.fromDegrees(300));
        }
        else if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 3600000) % 360 < 90){
          trapPose = new Pose2d(new Translation2d(Constants.Vision.TRAP_RIGHT_TARGET_X_BLUE, Constants.Vision.TRAP_RIGHT_TARGET_Y_BLUE), Rotation2d.fromDegrees(60));
        }
      }
      else{
        if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 3600000) % 360 > 90 || (Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 3600000) % 360 < 270){
          trapPose = new Pose2d(new Translation2d(Constants.Vision.TRAP_CENTER_TARGET_X_RED, Constants.Vision.TRAP_CENTER_TARGET_Y_RED), Rotation2d.fromDegrees(0));
        }
        else if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 3600000) % 360 > 270){
          trapPose = new Pose2d(new Translation2d(Constants.Vision.TRAP_LEFT_TARGET_X_RED, Constants.Vision.TRAP_LEFT_TARGET_Y_RED), Rotation2d.fromDegrees(300));
        }
        else if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 3600000) % 360 < 90){
          trapPose = new Pose2d(new Translation2d(Constants.Vision.TRAP_RIGHT_TARGET_X_RED, Constants.Vision.TRAP_RIGHT_TARGET_Y_RED), Rotation2d.fromDegrees(60));
        }
      }
      if(!scheduled){
        command = AutoBuilder.pathfindToPose(trapPose, constraints, 0, 0);
        command.schedule();
        scheduled = true;
      }
      if(scheduled && command.isFinished()){
        s_Swerve.drive(
              new Translation2d(translationVal, 0).times(Constants.Swerve.MAX_SPEED), 
              0, 
              robotCentricSup.getAsBoolean(), 
              true
          );
      }
    }
    else if(!RunElevator.deadShooter){
      /* Get Values, Deadband*/
      translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);

      if(robotCentricSup.getAsBoolean()){
        translationVal *= -1;
        strafeVal *= -1;
      }

      /* Drive */
      if(s_Swerve.getNoteTracking()){
        if(s_Vision.getNoteTargets()){
          s_Swerve.drive(
              new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
              s_Vision.getNoteRotationSpeed(), 
              !robotCentricSup.getAsBoolean(), 
              true
          );
        }
        else{
          s_Swerve.drive(
              new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
              rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY, 
              !robotCentricSup.getAsBoolean(), 
              true
          );
        }
      }
      else{
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
            s_Vision.getAprilTagRotationSpeed(), 
            !robotCentricSup.getAsBoolean(), 
            true
        );
      }

      if(s_Vision.getAprilTagRotationSpeed() < 0.3 && !flag){
        Robot.robotContainer.s_LED.reset();
        Robot.robotContainer.s_LED.green();
        flag = true;
      }
      if(s_Vision.getAprilTagRotationSpeed() >= 0.3){
        flag = false;
        Robot.robotContainer.s_LED.reset();
      }
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
