// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
  private final BooleanSupplier robotCentricSup;
  private boolean flag;
  private final PIDController posePid = new PIDController(0.3, 0, 0);
  private double translationVal, strafeVal, rotationVal;


  /** Creates a new Align. */
  public Align(Swerve s_Swerve, Vision s_Vision, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    this.s_Vision = s_Vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve, s_Vision);
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.robotCentricSup = robotCentricSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flag = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RunElevator.startStage1 || RunElevator.startStage2){
      strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      var alliance = DriverStation.getAlliance();
      if(alliance.isPresent()){
        if(alliance.get() == DriverStation.Alliance.Blue){
          translationVal = Constants.Vision.AMP_TARGET_X_BLUE - s_Swerve.getPose().getX();
          rotationVal = -posePid.calculate(Rotation2d.fromDegrees(270).getRadians() - s_Swerve.getGyroYaw().getRadians()) * Constants.Swerve.MAX_ANGULAR_VELOCITY;
        }
        else{
          translationVal = Constants.Vision.AMP_TARGET_X_RED - s_Swerve.getPose().getX();
          rotationVal = -posePid.calculate(Rotation2d.fromDegrees(90).getRadians() - s_Swerve.getGyroYaw().getRadians()) * Constants.Swerve.MAX_ANGULAR_VELOCITY;
        }
      }
      s_Swerve.drive(
          new Translation2d(translationVal, strafeVal), 
          rotationVal, 
          !robotCentricSup.getAsBoolean(), 
          false
      );
    }
    else if(RunElevator.climbing){
      translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      var alliance = DriverStation.getAlliance();
      if(alliance.isPresent()){
        if(alliance.get() == DriverStation.Alliance.Blue){
          if(Math.abs(Robot.robotContainer.s_Swerve.getHeading().getDegrees() % 360) > 90 || Math.abs(Robot.robotContainer.s_Swerve.getHeading().getDegrees() % 360) < 270){
            strafeVal = Constants.Vision.TRAP_CENTER_TARGET_Y_BLUE - s_Swerve.getPose().getY();
            rotationVal = -posePid.calculate(Rotation2d.fromDegrees(0).getRadians() - s_Swerve.getGyroYaw().getRadians()) * Constants.Swerve.MAX_ANGULAR_VELOCITY;
          }
          else if(Math.abs(Robot.robotContainer.s_Swerve.getHeading().getDegrees() % 360) > 300){
            strafeVal = Constants.Vision.TRAP_LEFT_TARGET_Y_BLUE - s_Swerve.getPose().getY();
            rotationVal = -posePid.calculate(Rotation2d.fromDegrees(120).getRadians() - s_Swerve.getGyroYaw().getRadians()) * Constants.Swerve.MAX_ANGULAR_VELOCITY;
          }
          else if(Math.abs(Robot.robotContainer.s_Swerve.getHeading().getDegrees() % 360) < 60){
            strafeVal = Constants.Vision.TRAP_RIGHT_TARGET_Y_BLUE - s_Swerve.getPose().getY();
            rotationVal = -posePid.calculate(Rotation2d.fromDegrees(240).getRadians() - s_Swerve.getGyroYaw().getRadians()) * Constants.Swerve.MAX_ANGULAR_VELOCITY;
          }
        }
        else{
          if(Math.abs(Robot.robotContainer.s_Swerve.getHeading().getDegrees() % 360) > 90 || Math.abs(Robot.robotContainer.s_Swerve.getHeading().getDegrees() % 360) < 270){
            strafeVal = Constants.Vision.TRAP_CENTER_TARGET_Y_RED - s_Swerve.getPose().getY();
            rotationVal = -posePid.calculate(Rotation2d.fromDegrees(0).getRadians() - s_Swerve.getGyroYaw().getRadians()) * Constants.Swerve.MAX_ANGULAR_VELOCITY;
          }
          else if(Math.abs(Robot.robotContainer.s_Swerve.getHeading().getDegrees() % 360) > 300){
            strafeVal = Constants.Vision.TRAP_LEFT_TARGET_Y_RED - s_Swerve.getPose().getY();
            rotationVal = -posePid.calculate(Rotation2d.fromDegrees(120).getRadians() - s_Swerve.getGyroYaw().getRadians()) * Constants.Swerve.MAX_ANGULAR_VELOCITY;
          }
          else if(Math.abs(Robot.robotContainer.s_Swerve.getHeading().getDegrees() % 360) < 60){
            strafeVal = Constants.Vision.TRAP_RIGHT_TARGET_Y_RED - s_Swerve.getPose().getY();
            rotationVal = -posePid.calculate(Rotation2d.fromDegrees(240).getRadians() - s_Swerve.getGyroYaw().getRadians()) * Constants.Swerve.MAX_ANGULAR_VELOCITY;
          }
        }
      }
      s_Swerve.drive(
          new Translation2d(translationVal, strafeVal), 
          rotationVal, 
          !robotCentricSup.getAsBoolean(), 
          false
      );
    }
    else if(!RunElevator.deadShooter){
      /* Get Values, Deadband*/
      translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);

      /* Drive */
      if(s_Swerve.getNoteTracking()){
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
            s_Vision.getNoteRotationSpeed(), 
            robotCentricSup.getAsBoolean(), 
            true
        );
      }
      else{
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
            s_Vision.getAprilTagRotationSpeed(), 
            !robotCentricSup.getAsBoolean(), 
            true
        );
      }

      if(s_Swerve.getSpeed().omegaRadiansPerSecond < 0.1 && !flag){
        Robot.robotContainer.s_LED.reset();
        Robot.robotContainer.s_LED.resetColors();
        Robot.robotContainer.s_LED.green();
        flag = true;
      }
      if(s_Swerve.getSpeed().omegaRadiansPerSecond >= 0.1){
        flag = false;
        Robot.robotContainer.s_LED.reset();
        Robot.robotContainer.s_LED.resetColors();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.robotContainer.s_LED.reset();
    Robot.robotContainer.s_LED.resetColors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
