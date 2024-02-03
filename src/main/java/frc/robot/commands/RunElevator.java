// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RunElevator extends Command {
  private final Elevator s_Elevator;
  private final Intake s_Intake;
  private final Shooter s_Shooter;
  private final XboxController xboxController;
  /** Creates a new RunElevator. */
  public RunElevator(Elevator s_Elevator, Intake s_Intake, Shooter s_Shooter, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Elevator = s_Elevator;
    this.s_Intake = s_Intake;
    this.s_Shooter = s_Shooter;
    this.xboxController = xboxController;
    addRequirements(s_Elevator, s_Intake, s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Elevator.setElevator(0); //FIXME setpoint
    s_Elevator.setBabyWrist(0); //FIXME setpoint
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Protect
    if(xboxController.getRawButton(0)){ //FIXME button and setpoints
      s_Elevator.setElevator(0);
      s_Elevator.setBabyWrist(0);
    }

    //Amp
    if(xboxController.getRawButton(0)){ //FIXME button and setpoints
      s_Elevator.setElevator(0);
      s_Elevator.setBabyWrist(0);
    }

    //Trap
    if(xboxController.getRawButton(0)){ //FIXME button and setpoints
      s_Elevator.setElevator(0);
      s_Elevator.setBabyWrist(0);
    }

    //Hand off
    if(xboxController.getRawButton(0)){ //FIXME button and setpoints
      s_Elevator.setElevator(0);
      s_Elevator.setBabyWrist(0);
    }

    //Spit for Hand off
    if(xboxController.getRawButton(0)){ //FIXME button and speed
      s_Elevator.runBabyShooterReverse();
      s_Shooter.runShooter(0);
      if(s_Intake.getBabyShooterSensor() == true && s_Intake.getShooterSensor() == false){
        s_Elevator.stopBabyShooter();
        s_Shooter.stopShooter();
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
