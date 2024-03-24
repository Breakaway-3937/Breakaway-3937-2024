// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RunNote extends Command {
  private final Intake s_Intake;
  private final Shooter s_Shooter;
  private final XboxController xboxController;
  private final double handoff = 18.3;
  private final double stageBackwards = 4;
  private final double protect = 13;
  /**
   * Formerly runIntakeBackwardsUntilShooterSensorReturnsAFalseValue and runIntakeBackwardsUntilIntakeSensorReturnsATrueValue
   */
  private boolean spitBack;
  private boolean deadIntake, sendForward;
  public static boolean noteGood;

  /** Creates a new RunNote. */
  public RunNote(Intake s_Intake, Shooter s_Shooter, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = s_Intake;
    this.s_Shooter = s_Shooter;
    this.xboxController = xboxController;
    addRequirements(s_Intake, s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Shooter.setWrist(protect);
    spitBack = false;
    deadIntake = false;
    sendForward = false;
    if(s_Intake.botFull()){
      noteGood = true;
    }
    else{
      noteGood = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RunElevator.handoff){
      s_Shooter.setWrist(handoff);
    }

    if(RunElevator.trapStage1){
      s_Intake.intakeSlowly();
      s_Shooter.setShooter(10);
      deadIntake = false;
      noteGood = false;
      spitBack = false;
      sendForward = false;
    }
    else if(RunElevator.trapStage2){
      s_Intake.stop();
      s_Shooter.stopShooter();
      deadIntake = false;
      noteGood = true;
      spitBack = false;
      sendForward = false;
    }
    else if(RunElevator.startStage1){
      if(s_Shooter.isAtPosition()){
        s_Intake.intakeSlowly();
        s_Shooter.setShooter(5);
      }
    }
    else if(RunElevator.startStage2){
      s_Intake.stop();
      s_Shooter.stopShooter();
      deadIntake = false;
      noteGood = false;
      sendForward = false;
      spitBack = false;
      s_Shooter.setWrist(protect);
    }
    else if(RunElevator.reverse){
      s_Intake.spit();
      s_Shooter.setShooter(-10);
      spitBack = true;
    }
    else{
      //No Buttons
      if(!spitBack && xboxController.getLeftTriggerAxis() <= 0.3 && !xboxController.getRawButton(5) && xboxController.getRightTriggerAxis() <= 0.3){
        s_Intake.stop();
      }
      //Intake
      else if(xboxController.getLeftTriggerAxis() > 0.3 && !deadIntake){
        s_Intake.intake();
      }
      //Spit
      else if(xboxController.getRawButton(5)){
        deadIntake = false;
        noteGood = false;
        spitBack = false;
        sendForward = false;
        if(s_Shooter.isAtPosition()){
          s_Intake.spit();
          s_Shooter.setShooter(-10);
        }
        else{
          s_Intake.stop();
          s_Shooter.stopShooter();
        }
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

      //Run Shooter
      if(xboxController.getRawButton(6)){
        s_Shooter.runShooter();
        if(!RunElevator.deadShooter){
          s_Shooter.setWristShooting();
        }
      }
      //Stop Shooter
      else if(!xboxController.getRawButton(6)){
        Shooter.sixMoreSmidgens = false;
        Shooter.shooterSad = false;
        if(!xboxController.getRawButton(5)){
          s_Shooter.stopShooter();
        }
        s_Shooter.setSpeedToZero();
        if(xboxController.getRawButton(5)){
          RunElevator.handoff = false;
        }
        
        //Pre-Stages after it gets note
        if(!RunElevator.deadShooter && s_Intake.botFull() && !s_Intake.getBabyShooterSensor() && !xboxController.getRawButton(5) && noteGood){
          if(Robot.getFront()){
            s_Shooter.setWrist(handoff);
            RunElevator.handoff = true;
          }
          else{
            s_Shooter.setWrist(stageBackwards);
            RunElevator.handoff = false;
          }
        }
        else if(!RunElevator.handoff){
          s_Shooter.setWrist(protect);
        }
      }

      //Fire Shooter
      if(s_Shooter.atSpeed() && s_Shooter.isAtPosition() && xboxController.getRightTriggerAxis() > 0.3){
        s_Intake.intake();
        deadIntake = false;
        noteGood = false;
        spitBack = false;
        sendForward = false;
      }
      //Sensor Detects, Stop Intake
      else if(xboxController.getRightTriggerAxis() <= 0.3 && s_Intake.getShooterSensor() && !noteGood){
        spitBack = true;
      }
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
