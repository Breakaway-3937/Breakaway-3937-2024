// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class RunElevator extends Command {
  private final Elevator s_Elevator;
  private final XboxController xboxController;
  //FIXME get all setpoints
  private final double wristHandoff = 0;
  private final double wristProtect = 0;
  private final double wristAmp = 0;
  private final double wristTrap = 0;
  private final double wristSource = 0;
  private final double elevatorHandoff = 0;
  private final double elevatorProtect = 0;
  private final double elevatorAmp = 0;
  private final double elevatorTrap = 0;
  private final double elevatorSource = 0;
  private boolean amp, trap, source, protect, ohCrap, climb;
  public static boolean deadShooter, trapStage1, trapStage2, startStage1, startStage2, reverse, trapScored, retracting;

  /** Creates a new RunElevator. */
  public RunElevator(Elevator s_Elevator, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Elevator = s_Elevator;
    this.xboxController = xboxController;
    addRequirements(s_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    protect = true;
    amp = false;
    trap = false;
    source = false;
    ohCrap = false;
    trapStage1 = false;
    trapStage2 = false;
    startStage1 = false;
    startStage2 = false;
    deadShooter = false;
    reverse = false;
    climb = false;
    trapScored = false;
    retracting = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Extend Climb
    if(xboxController.getPOV() == Constants.Controllers.UP){
      amp = false;
      trap = false;
      source = false;
      protect = false;
      ohCrap = false;
      trapStage1 = false;
      trapStage2 = false;
      startStage1 = false;
      startStage2 = false;
      deadShooter = true;
      reverse = false;
      climb = true;
      retracting = false;
    }
    //Retract Climb
    else if(xboxController.getPOV() == Constants.Controllers.DOWN){
      amp = false;
      trap = false;
      source = false;
      protect = false;
      ohCrap = false;
      trapStage1 = false;
      trapStage2 = false;
      startStage1 = false;
      startStage2 = false;
      deadShooter = false;
      reverse = false;
      climb = false;
      retracting = true;
    }
    //Protect
    else if(xboxController.getRawButton(3)){
      amp = false;
      trap = false;
      source = false;
      protect = true;
      ohCrap = false;
      trapStage1 = false;
      trapStage2 = false;
      startStage1 = false;
      startStage2 = false;
      deadShooter = false;
      reverse = false;
      climb = false;
      retracting = false;
    }
    //Oh Crap!
    else if(xboxController.getRawButton(2)){
      amp = false;
      trap = false;
      source = false;
      protect = false;
      ohCrap = true;
      trapStage1 = false;
      trapStage2 = false;
      startStage1 = false;
      startStage2 = false;
      deadShooter = true;
      reverse = false;
      climb = false;
      retracting = false;
    }
    //Amp
    else if(xboxController.getRawButton(1)){
      amp = true;
      trap = false;
      source = false;
      protect = false;
      ohCrap = false;
      trapStage1 = false;
      trapStage2 = false;
      startStage1 = false;
      startStage2 = false;
      deadShooter = true;
      reverse = false;
      climb = false;
      retracting = false;
    }
    //Trap
    else if(xboxController.getRawButton(4)){
      amp = false;
      trap = true;
      source = false;
      protect = false;
      ohCrap = false;
      startStage1 = false;
      startStage2 = false;
      deadShooter = true;
      reverse = false;
      climb = false;
      retracting = false;
    }
    //Source
    else if(xboxController.getRawButton(7)){
      amp = false;
      trap = false;
      source = true;
      protect = false;
      ohCrap = false;
      trapStage1 = false;
      trapStage2 = false;
      startStage1 = false;
      startStage2 = false;
      deadShooter = true;
      reverse = false;
      climb = false;
      retracting = false;
      Robot.robotContainer.s_LED.resetColors();
      Robot.robotContainer.s_LED.orange();
    }
    
    if(Robot.robotContainer.s_Shooter.isSafe()){
      if(protect){
        s_Elevator.setElevator(elevatorProtect);
        s_Elevator.setBabyWrist(wristProtect);
      }
      else if(trap){
        if(!trapStage2 && !Robot.robotContainer.s_Intake.getBabyShooterSensor() && (Robot.robotContainer.s_Intake.getShooterSensor() || Robot.robotContainer.s_Intake.getIntakeSensor())){
          s_Elevator.setElevator(elevatorHandoff);
          s_Elevator.setBabyWrist(wristHandoff);
        }
        if(!trapStage2 && s_Elevator.isAtPosition() && !Robot.robotContainer.s_Intake.getBabyShooterSensor() && (Robot.robotContainer.s_Intake.getShooterSensor() || Robot.robotContainer.s_Intake.getIntakeSensor())){
          s_Elevator.runBabyShooterForward();
          trapStage1 = true;
        }
        if(Robot.robotContainer.s_Intake.getBabyShooterSensor() && (!Robot.robotContainer.s_Intake.getShooterSensor() || !Robot.robotContainer.s_Intake.getIntakeSensor())){
          s_Elevator.stopBabyShooter();
          trapStage1 = false;
          trapStage2 = true;
        }
        if(trapStage2){
          s_Elevator.setElevator(elevatorTrap);
          s_Elevator.setBabyWrist(wristTrap);
        }
        if(trapStage2 && s_Elevator.isAtPosition() && Robot.robotContainer.s_Intake.getBabyShooterSensor()){
          s_Elevator.runBabyShooterForward();
        }
        else if(trapStage2 && s_Elevator.isAtPosition() && !Robot.robotContainer.s_Intake.getBabyShooterSensor()){
          s_Elevator.stopBabyShooter();
          trapScored = true;
        }
      }
      else if(amp){
        if(!startStage1 && !startStage2){
          s_Elevator.setElevator(elevatorHandoff);
          s_Elevator.setBabyWrist(wristHandoff);
        }
        if(xboxController.getRawButton(8) && !startStage2){
          s_Elevator.runBabyShooterForward();
          startStage1 = true;
        }
        if(Robot.robotContainer.s_Intake.getBabyShooterSensor() && (!Robot.robotContainer.s_Intake.getShooterSensor() || !Robot.robotContainer.s_Intake.getIntakeSensor())){
          s_Elevator.stopBabyShooter();
          startStage1 = false;
          startStage2 = true;
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
        }
      }
      else if(ohCrap){
        if(Robot.robotContainer.s_Intake.getBabyShooterSensor() && (!Robot.robotContainer.s_Intake.getShooterSensor() || !Robot.robotContainer.s_Intake.getIntakeSensor())){
          s_Elevator.setElevator(elevatorHandoff);
          s_Elevator.setBabyWrist(wristHandoff);
        }
        if(s_Elevator.isAtPosition() && Robot.robotContainer.s_Intake.getBabyShooterSensor() && (!Robot.robotContainer.s_Intake.getShooterSensor() || !Robot.robotContainer.s_Intake.getIntakeSensor())){
          reverse = true;
          s_Elevator.runBabyShooterReverse();
        }
        if(!Robot.robotContainer.s_Intake.getBabyShooterSensor() && (Robot.robotContainer.s_Intake.getShooterSensor() || Robot.robotContainer.s_Intake.getIntakeSensor())){
          reverse = false;
          s_Elevator.stopBabyShooter();
          protect = true;
          deadShooter = false;
          ohCrap = false;
        }
      }
      else if(source){
        if(!Robot.robotContainer.s_Intake.botFull()){
          s_Elevator.setElevator(elevatorSource);
          s_Elevator.setBabyWrist(wristSource);
          s_Elevator.runBabyShooterReverse();
        }
        if(Robot.robotContainer.s_Intake.getBabyShooterSensor()){
          s_Elevator.stopBabyShooter();
          ohCrap = true;
          source = false;
        }
      }
      else if(climb){
        s_Elevator.extendClimb();
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
