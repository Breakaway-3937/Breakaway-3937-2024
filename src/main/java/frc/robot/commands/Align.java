// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class Align extends Command {
  private final CommandSwerveDrivetrain s_Swerve;
  private final Vision s_Vision;
  private final DoubleSupplier translationSup;
  private final DoubleSupplier strafeSup;
  private final DoubleSupplier rotationSup;
  private final BooleanSupplier robotCentricSup;
  private final double pValue = 8.0 / 42.0;
  private boolean flag;
  private double translationVal, strafeVal, rotationVal;
  private SwerveRequest.FieldCentricFacingAngle swerveRequest = new SwerveRequest.FieldCentricFacingAngle()
                                                                      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                                                                      .withSteerRequestType(SteerRequestType.MotionMagic)
                                                                      .withVelocityX(0)
                                                                      .withVelocityY(0);

  /** Creates a new Align. */
  public Align(CommandSwerveDrivetrain s_Swerve, Vision s_Vision, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    this.s_Vision = s_Vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve, s_Vision);
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;

    swerveRequest.HeadingController = new PhoenixPIDController(5, 0, 0);
    //swerveRequest.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flag = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if(RunElevator.amp && Robot.robotContainer.s_Intake.botFull()){
      translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      if(!Robot.getRedAlliance()){
        rotationVal = pValue * (90 - (s_Swerve.getPigeon2().getAngle() + 3600000) % 360);
      }
      else{
        rotationVal = pValue * (270 - (s_Swerve.getPigeon2().getAngle() + 3600000) % 360);
      }
      s_Swerve.applyRequest()
    }
    else if(RunElevator.climbing){
      translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 3600000) % 360 > 90 && (Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 3600000) % 360 < 270){
        rotationVal = pValue * (180 - (s_Swerve.getGyroYaw().getDegrees() + 3600000) % 360);
      }
      else if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 3600000) % 360 > 270){
        rotationVal = pValue * (300 - (s_Swerve.getGyroYaw().getDegrees() + 3600000) % 360);
      }
      else if((Robot.robotContainer.s_Swerve.getHeading().getDegrees() + 3600000) % 360 < 90){
        rotationVal = pValue * (60 - (s_Swerve.getGyroYaw().getDegrees() + 3600000) % 360);
      }
      s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
            rotationVal, 
            !robotCentricSup.getAsBoolean(),
            true
        );
    }
    else if(!RunElevator.deadShooter || (RunElevator.amp && !Robot.robotContainer.s_Intake.botFull())){
      /* Get Values, Deadband*/
      /*translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
      rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);

      if(robotCentricSup.getAsBoolean()){
        translationVal *= -1;
        strafeVal *= -1;
      }*/

      /* Drive */
      /*if(s_Swerve.getNoteTracking()){
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

      if(Math.abs(s_Vision.getAprilTagYaw()) < 3 && !flag){
        Robot.robotContainer.s_LED.reset();
        Robot.robotContainer.s_LED.green();
        flag = true;
      }
      if(Math.abs(s_Vision.getAprilTagYaw()) >= 3){
        flag = false;
        Robot.robotContainer.s_LED.reset();
      }
    }*/
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
