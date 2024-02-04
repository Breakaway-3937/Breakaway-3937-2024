// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

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
  private double elevatorNext = 0;
  private double wristNext = 0;
  private boolean amp, trap, source, protect;

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
    elevatorNext = elevatorProtect;
    wristNext = wristProtect;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Protect
    if(xboxController.getRawButton(3)){
      elevatorNext = elevatorProtect;
      wristNext = wristProtect;
      amp = false;
      trap = false;
      source = false;
      protect = true;
    }
    //Oh Crap!
    else if(xboxController.getRawButton(2)){
      elevatorNext = elevatorProtect;
      wristNext = wristProtect;
      amp = false;
      trap = false;
      source = false;
      protect = false;
    }
    //Amp
    else if(xboxController.getRawButton(1)){
      elevatorNext = elevatorAmp;
      wristNext = wristAmp;
      amp = true;
      trap = false;
      source = false;
      protect = false;
    }
    //Trap
    else if(xboxController.getRawButton(4)){
      elevatorNext = elevatorTrap;
      wristNext = wristTrap;
      amp = false;
      trap = true;
      source = false;
      protect = false;
    }
    //Source
    else if(xboxController.getRawButton(7)){
      elevatorNext = elevatorSource;
      wristNext = wristSource;
      amp = false;
      trap = false;
      source = true;
      protect = false;
    }
    
    if(Shooter.isSafe()){
      if(protect){
        s_Elevator.setElevator(elevatorNext);
        s_Elevator.setBabyWrist(wristNext);
      }
      else if(trap){
        s_Elevator.setElevator(elevatorHandoff);
        s_Elevator.setBabyWrist(wristHandoff);
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
