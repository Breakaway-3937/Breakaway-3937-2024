// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
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
  private final BooleanSupplier robotCentricSup;
  private boolean flag;

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
    /* Get Values, Deadband*/
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Controllers.STICK_DEADBAND);

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
      Robot.robotContainer.s_LED.resetColors();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.robotContainer.s_LED.resetColors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
