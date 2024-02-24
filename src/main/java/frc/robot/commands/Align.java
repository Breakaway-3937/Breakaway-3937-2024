// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
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
  private final double pValue = 8.0 / 42.0;
  private boolean flag;
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
    if(RunElevator.amp){
      translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      var alliance = DriverStation.getAlliance();
      if(alliance.isPresent()){
        if(alliance.get() == DriverStation.Alliance.Blue){
          rotationVal = pValue * (90 - (s_Swerve.getGyroYaw().getDegrees() + 3600000) % 360);
        }
        else{
          rotationVal = pValue * (270 - (s_Swerve.getGyroYaw().getDegrees() + 3600000) % 360);
        }
      }
      s_Swerve.drive(
          new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
          rotationVal, 
          !robotCentricSup.getAsBoolean(), 
          true
      );
    }
    else if(RunElevator.climbing){
      translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      var alliance = DriverStation.getAlliance();
      if(alliance.isPresent()){
        if(alliance.get() == DriverStation.Alliance.Blue){
          if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 360000000) % 360 > 90 || (Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 360000000) % 360 < 270){
            strafeVal = Constants.Vision.TRAP_CENTER_TARGET_Y_BLUE - s_Swerve.getPose().getY();
            rotationVal = pValue * (Rotation2d.fromDegrees(0).getRadians() - (s_Swerve.getGyroYaw().getRadians() + (Math.PI * 500)) % (Math.PI * 2));
          }
          else if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 360000000) % 360 > 300){
            strafeVal = Constants.Vision.TRAP_LEFT_TARGET_Y_BLUE - s_Swerve.getPose().getY();
            rotationVal = pValue * (Rotation2d.fromDegrees(120).getRadians() - (s_Swerve.getGyroYaw().getRadians() + (Math.PI * 500)) % (Math.PI * 2));
          }
          else if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 360000000) % 360 < 60){
            strafeVal = Constants.Vision.TRAP_RIGHT_TARGET_Y_BLUE - s_Swerve.getPose().getY();
            rotationVal = pValue * (Rotation2d.fromDegrees(240).getRadians() - (s_Swerve.getGyroYaw().getRadians() + (Math.PI * 500)) % (Math.PI * 2));
          }
        }
        else{
          if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 360000000) % 360 > 90 || (Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 360000000) % 360 < 270){
            strafeVal = Constants.Vision.TRAP_CENTER_TARGET_Y_RED - s_Swerve.getPose().getY();
            rotationVal = pValue * (Rotation2d.fromDegrees(0).getRadians() - (s_Swerve.getGyroYaw().getRadians() + (Math.PI * 500)) % (Math.PI * 2));
          }
          else if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 360000000) % 360 > 300){
            strafeVal = Constants.Vision.TRAP_LEFT_TARGET_Y_RED - s_Swerve.getPose().getY();
            rotationVal = pValue * (Rotation2d.fromDegrees(120).getRadians() - (s_Swerve.getGyroYaw().getRadians() + (Math.PI * 500)) % (Math.PI * 2));
          }
          else if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 360000000) % 360 < 60){
            strafeVal = Constants.Vision.TRAP_RIGHT_TARGET_Y_RED - s_Swerve.getPose().getY();
            rotationVal = pValue * (Rotation2d.fromDegrees(240).getRadians() - (s_Swerve.getGyroYaw().getRadians() + (Math.PI * 500)) % (Math.PI * 2));
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
            !robotCentricSup.getAsBoolean(), 
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
        Robot.robotContainer.s_LED.green();
        flag = true;
      }
      if(s_Swerve.getSpeed().omegaRadiansPerSecond >= 0.1){
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
