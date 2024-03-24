// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

public class RunElevator extends Command {
  private final Elevator s_Elevator;
  private final XboxController xboxController;
  private final double wristHandoff = 65.6;
  private final double wristProtect = 3;
  private final double wristSixMoreSmidgens = 6.5;
  private final double wristAmp = 45.5;
  private final double wristDoublePreTrap = 51;
  private final double wristPreTrap = 60;
  private final double wristTrap = 37.15;
  private final double wristSource = 65;
  private final double elevatorHandoff = 28.2;
  private final double elevatorProtect = 0;
  private final double elevatorAmp = 80.6;
  private final double elevatorTrap = 102;
  private final double elevatorSource = 52.6;
  private final double elevatorClimb = 102;
  private boolean trap, source, protect, ohCrap, climb;
  public static boolean amp, deadShooter, trapStage1, trapStage2, startStage1, startStage2, reverse, trapScored, retracting, climbing, handoff, trapPosition;
  private final Timer timer;

  /** Creates a new RunElevator. */
  public RunElevator(Elevator s_Elevator, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Elevator = s_Elevator;
    this.xboxController = xboxController;
    timer = new Timer();
    addRequirements(s_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    reset();
    protect = true;
    Robot.robotContainer.s_LED.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(protect && s_Elevator.isAtPosition() && s_Elevator.getElevator() < 7.5){
      deadShooter = false;
    }
    else{
      deadShooter = true;
    }

    //Extend Climb
    if(xboxController.getPOV() == Constants.Controllers.UP){
      reset();
      climb = true;
      climbing = true;
      trap = true;
    }
    //Retract Climb
    else if(xboxController.getPOV() == Constants.Controllers.DOWN){
      reset();
      climb = true;
      retracting = true;
      trap = true;
    }
    //Protect
    else if(xboxController.getRawButton(3)){
      reset();
      protect = true;
      trapScored = false;
      Robot.robotContainer.s_LED.reset();
      s_Elevator.setElevatorFast();
      s_Elevator.stopBabyShooter();
    }
    //Oh Crap!
    else if(xboxController.getRawButton(2)){
      reset();
      ohCrap = true;
    }
    //Amp
    else if(xboxController.getRawButton(1)){
      reset();
      amp = true;
    }
    //Trap
    else if(xboxController.getRawButton(4)){
      reset();
      trap = true;
    }
    //Source
    else if(xboxController.getRawButton(7)){
      reset();
      source = true;
      Robot.robotContainer.s_LED.reset();
      Robot.robotContainer.s_LED.orange();
    }
    
    if(Robot.robotContainer.s_Shooter.isSafe() || handoff){
      if(protect){
        s_Elevator.setElevator(elevatorProtect);
        if(Shooter.sixMoreSmidgens){
          s_Elevator.setBabyWrist(wristSixMoreSmidgens);
        }
        else{
          s_Elevator.setBabyWrist(wristProtect);
        }
        handoff = false;
      }
      else if(trap && !climb){
        s_Elevator.setElevator(elevatorHandoff);
        s_Elevator.setBabyWrist(wristHandoff);
        if(s_Elevator.isAtPosition()){
          handoff = true;
        }
        if(handoff && !Robot.robotContainer.s_Shooter.isSafe() && Robot.robotContainer.s_Shooter.isAtPosition() && s_Elevator.isAtPosition() && !Robot.robotContainer.s_Intake.getBabyShooterSensor()){
          s_Elevator.runBabyShooterForwardSlowly();
          trapStage1 = true;
        }
        if(Robot.robotContainer.s_Intake.getBabyShooterSensor()){
          s_Elevator.stopBabyShooter();
          trapStage1 = false;
          trapStage2 = true;
        }
      }
      else if(amp){
        if(!startStage1 && !startStage2){
          s_Elevator.setElevator(elevatorHandoff);
          s_Elevator.setBabyWrist(wristHandoff);
          if(RunNote.noteGood){
            handoff = true;
          }
        }
        if(xboxController.getRawButton(8) && !startStage2){
          s_Elevator.runBabyShooterForward();
          startStage1 = true;
        }
        if(Robot.robotContainer.s_Intake.getBabyShooterSensor() && !Robot.robotContainer.s_Intake.getShooterSensor() && !Robot.robotContainer.s_Intake.getPopTartSensor()){
          s_Elevator.stopBabyShooter();
          startStage1 = false;
          startStage2 = true;
          handoff = false;
        }
        if(startStage2){
          s_Elevator.setElevator(elevatorAmp);
          s_Elevator.setBabyWrist(wristAmp);
          if(xboxController.getRightTriggerAxis() > 0.3){
            s_Elevator.runBabyShooterForward();
          }
          else{
            s_Elevator.stopBabyShooter();
          }
          if(Robot.robotContainer.s_Intake.getShooterSensor()){
            s_Elevator.setElevator(elevatorHandoff);
            s_Elevator.setBabyWrist(wristHandoff);
            s_Elevator.runBabyShooterForward();
            startStage2 = false;
            startStage1 = true;
          }
        }
      }
      else if(ohCrap){
        if(Robot.robotContainer.s_Intake.getBabyShooterSensor()){
          s_Elevator.setElevator(elevatorHandoff);
          s_Elevator.setBabyWrist(wristHandoff);
          reverse = true;
          handoff = true;
        }
        if(Robot.robotContainer.s_Shooter.isAtPosition() && s_Elevator.isAtPosition() && Robot.robotContainer.s_Intake.getBabyShooterSensor()){
          s_Elevator.runBabyShooterReverse();
        }
        if(!Robot.robotContainer.s_Intake.getBabyShooterSensor() && (Robot.robotContainer.s_Intake.getShooterSensor() || Robot.robotContainer.s_Intake.getPopTartSensor())){
          reverse = false;
          handoff = false;
          s_Elevator.stopBabyShooter();
          protect = true;
          ohCrap = false;
        }
      }
      else if(source){
        s_Elevator.setElevator(elevatorSource);
        s_Elevator.setBabyWrist(wristSource);
        s_Elevator.runBabyShooterReverse();

        if(Robot.robotContainer.s_Intake.getBabyShooterSensor()){
          s_Elevator.stopBabyShooter();
          Robot.robotContainer.s_LED.reset();
          ohCrap = true;
          source = false;
        }
      }
      
      if(climb){
        if(climbing){
          s_Elevator.setElevatorFast();
          s_Elevator.setBabyWrist(wristHandoff);
          s_Elevator.setElevator(elevatorClimb);
        }
        else if(retracting && !trapStage2){
          s_Elevator.setElevatorSlow();
          s_Elevator.setBabyWrist(wristDoublePreTrap);
          s_Elevator.setElevator(elevatorProtect);
        }

        if(trap){
          if(xboxController.getRawButton(8)){
            trapStage2 = true;
            s_Elevator.setElevatorEvenSlower();
            s_Elevator.setBabyShooterBrake();
          }
          if(trapStage2 && !trapPosition){
            s_Elevator.setElevator(elevatorTrap);
            s_Elevator.setBabyWrist(wristPreTrap);
            if(s_Elevator.getElevator() < 40.0){
              s_Elevator.setBabyShooterBrake();
            }
            else{
              s_Elevator.setBabyShooterBrake();
            }
          }
          if(trapStage2 && s_Elevator.isAtPosition()){
            if(s_Elevator.isAtPosition()){
              s_Elevator.setBabyWrist(wristTrap);
              trapPosition = true;
            }
            if(s_Elevator.isAtPosition() && trapPosition){
              s_Elevator.setBabyShooterCoast();
              s_Elevator.runBabyShooterForward();
              timer.start();
            }
          }
          if(trapPosition && !Robot.robotContainer.s_Intake.getBabyShooterSensor() && timer.get() > 2){
            trapScored = true;
            s_Elevator.stopBabyShooter();
            s_Elevator.setBabyWrist(wristHandoff);
          }
        }
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

  private void reset(){
    protect = false;
    amp = false;
    trap = false;
    source = false;
    ohCrap = false;
    trapStage1 = false;
    trapStage2 = false;
    startStage1 = false;
    startStage2 = false;
    reverse = false;
    climb = false;
    retracting = false;
    climbing = false;
    handoff = false;
    trapPosition = false;
    timer.stop();
    timer.reset();
  }
}
