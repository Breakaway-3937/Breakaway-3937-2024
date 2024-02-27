// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoRunNote extends Command {
  private final Intake s_Intake;
  private final Shooter s_Shooter;
  private final double protect = 13;
  private boolean spitBack, deadIntake, sendForward, noteGood;

  /** Creates a new AutoRunNote. */
  public AutoRunNote(Intake s_Intake, Shooter s_Shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = s_Intake;
    this.s_Shooter = s_Shooter;
    addRequirements(s_Intake, s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Shooter.setWrist(protect);
    spitBack = false;
    deadIntake = false;
    sendForward = false;
    noteGood = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //No Commands
    if(!spitBack && !s_Intake.autoIntake() && !s_Shooter.autoFire()){
      s_Intake.stop();
    }
    //Intake
    else if(s_Intake.autoIntake() && !deadIntake){
      s_Intake.intake();
    }

    if(spitBack){
      s_Intake.spitSlowly();
      s_Shooter.setShooter(-10);
    }
    if(spitBack && s_Intake.getPopTartSensor()){
      spitBack = false;
      deadIntake = true;
      sendForward = true;
      s_Shooter.stopShooter();
    }

    if(sendForward){
      s_Intake.intakeSlowly();
    }
    if(sendForward && !s_Intake.getPopTartSensor()){
      sendForward = false;
      noteGood = true;
      s_Intake.stop();
    }

    //Shoot
    if(s_Shooter.autoFire()){
      s_Shooter.runShooter();
      s_Shooter.setWristShooting();
      if(s_Shooter.atSpeed() && s_Shooter.isAtPosition()){
        s_Intake.intake();
        deadIntake = false;
        noteGood = false;
        sendForward = false;
        spitBack = false;
      }
      if(!s_Intake.botFull()){
        s_Shooter.setAutoFire(false);
        s_Intake.setAutoIntake(false);
      }
    }
    //Sensor Detects, Stop Intake
    else if(!s_Shooter.autoFire() && s_Intake.getShooterSensor() && !noteGood){
      spitBack = true;
    }

    //Wrist Protect
    if(!s_Shooter.autoFire()){
      s_Shooter.setWrist(protect);
      Shooter.sixMoreSmidgens = false;
    }
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
