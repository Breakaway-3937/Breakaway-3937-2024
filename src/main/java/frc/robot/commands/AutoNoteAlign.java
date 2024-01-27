// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AutoNoteAlign extends Command {
  private Swerve s_Swerve;
  private Vision s_Vision;
  private Timer time;
  private boolean done;
  private double lastRotationSpeed;

  /** Creates a new AutoNoteAlign. */
  public AutoNoteAlign(Swerve s_Swerve, Vision s_Vision) {
    this.s_Swerve = s_Swerve;
    this.s_Vision = s_Vision;
    time = new Timer();
    time.reset();
    done = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve, s_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double translationXValue = 1;
    double translationYValue = 0;

    if(s_Vision.getNoteTargets() == true){
      s_Swerve.drive(new Translation2d(translationXValue, translationYValue), s_Vision.getNoteRotationSpeed(), false, false);
      System.out.println("SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET SEE TARGET ");
    }
    else{
      lastRotationSpeed = s_Vision.getNoteRotationSpeed();
      time.start();
      if(time.get() < 3){
        s_Swerve.drive(new Translation2d(translationXValue, translationYValue), lastRotationSpeed, false, false);
        System.out.println("TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER TIMER ");
      }
      else{
        System.out.println("END END END END END END END END END END END END END END END END END END END END END END END END ");
        done = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
