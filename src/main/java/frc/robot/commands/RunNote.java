// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RunNote extends Command {
  private final Intake s_Intake;
  private final Shooter s_Shooter;
  private final XboxController xboxController;
  private final double handoff = 17.6;
  private final double protect = 13;
  /**
   * Formerly runIntakeBackwardsUntilShooterSensorReturnsAFalseValue and runIntakeBackwardsUntilIntakeSensorReturnsATrueValue
   */
  private boolean spitBack;
  private boolean deadIntake, sendForward, noteGood;

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
    noteGood = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RunElevator.handoff){
      s_Shooter.setWrist(handoff);
      Shuffleboard.selectTab("Elevator");
    }

    if(RunElevator.trapStage1){
      s_Intake.intake();
      s_Shooter.setShooter(5);
      deadIntake = false;
      noteGood = false;
      spitBack = false;
      sendForward = false;
      Shuffleboard.selectTab("Elevator");
    }
    else if(RunElevator.trapStage2){
      s_Intake.stop();
      s_Shooter.stopShooter();
      deadIntake = false;
      noteGood = false;
      spitBack = false;
      sendForward = false;
      Shuffleboard.selectTab("Elevator");
    }
    else if(RunElevator.startStage1){
      if(s_Shooter.isAtPosition()){
        s_Intake.intake();
        s_Shooter.setShooter(5);
        Shuffleboard.selectTab("Elevator");
      }
    }
    else if(RunElevator.startStage2){
      s_Intake.stop();
      s_Shooter.stopShooter();
      s_Shooter.setWrist(protect);
      deadIntake = false;
      noteGood = false;
      sendForward = false;
      spitBack = false;
      Shuffleboard.selectTab("Elevator");
    }
    else if(RunElevator.reverse){
      s_Intake.spit();
      s_Shooter.setShooter(-10);
      spitBack = true;
      Shuffleboard.selectTab("Elevator");
    }
    else{
      //No Buttons
      if(!spitBack && xboxController.getLeftTriggerAxis() <= 0.3 && !xboxController.getRawButton(5) && xboxController.getRightTriggerAxis() <= 0.3){
        s_Intake.stop();
        Shuffleboard.selectTab("Drive");
      }
      //Intake
      else if(xboxController.getLeftTriggerAxis() > 0.3 && !deadIntake){
        s_Intake.intake();
        Shuffleboard.selectTab("Intake");
      }
      //Spit
      else if(xboxController.getRawButton(5)){
        deadIntake = false;
        noteGood = false;
        spitBack = false;
        sendForward = false;
        s_Intake.spit();
        s_Shooter.setShooter(-10);
        Shuffleboard.selectTab("Intake");
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
      if(sendForward && s_Intake.getShooterSensor()){
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
        Shuffleboard.selectTab("Shooter");
      }
      //Stop Shooter
      else{
        s_Shooter.stopShooter();
        s_Shooter.setSpeedToZero();
        if(s_Intake.botFull() && !s_Intake.getBabyShooterSensor() && !xboxController.getRawButton(5) && noteGood){
          s_Shooter.setWrist(handoff);
          RunElevator.handoff = true;
        }
        else if(!RunElevator.handoff){
          s_Shooter.setWrist(protect);
          RunElevator.handoff = false;
        }
        Shuffleboard.selectTab("Drive");
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
      else if(xboxController.getRightTriggerAxis() <= 0.3 && s_Intake.getShooterSensor() && !noteGood && !RunElevator.trap){
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
