// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AutoNoteAlign extends Command {
  private final Swerve s_Swerve;
  private final Vision s_Vision;
  private final Timer timer;
  private boolean done;
  private final double translationXValue = -3;

  /** Creates a new AutoNoteAlign. */
  public AutoNoteAlign(Swerve s_Swerve, Vision s_Vision) {
    this.s_Swerve = s_Swerve;
    this.s_Vision = s_Vision;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve, s_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_Vision.getNoteTargets() && s_Vision.getNoteLatency() != 0){
      s_Swerve.drive(new Translation2d(translationXValue, 0), s_Vision.getNoteRotationSpeed(), false, false);
    }
    else{
      timer.start();
    }

    if(timer.get() < 1 && (s_Swerve.getPose().getX() < 8.6 && !Robot.getRedAlliance() || s_Swerve.getPose().getX() > 8.6 && Robot.getRedAlliance())){
      s_Swerve.drive(new Translation2d(translationXValue, 0), 0, false, false);
    }
    else{
      done = true;
    }
  }

  // Called once the command ends or is interrupted.m
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
