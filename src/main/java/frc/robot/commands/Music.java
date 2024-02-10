// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;

public class Music extends Command {
  private final Swerve s_Swerve;
  private final Orchestra orchestra;
  private int count = 0;
  private double time = 0;
  /** Creates a new Music. */
  public Music(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    orchestra = new Orchestra();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = (int) (Math.random() * 11);
    for(int i = 0; i < 4; i++){
      orchestra.addInstrument(s_Swerve.mSwerveMods[i].getDriveMotor());
      orchestra.addInstrument(s_Swerve.mSwerveMods[i].getAngleMotor());
    }
    orchestra.addInstrument(Robot.robotContainer.s_Intake.getLoaderMotor());
    orchestra.addInstrument(Robot.robotContainer.s_Shooter.getShooterMotors().getFirst());
    orchestra.addInstrument(Robot.robotContainer.s_Shooter.getShooterMotors().getSecond());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(orchestra.getCurrentTime() >= time){
      if(count == 0){
        orchestra.loadMusic("GodBless.chrp");
        count++;
        time = 99;
        orchestra.play();
      }
      else if(count == 1){
        orchestra.loadMusic("Champions.chrp");
        count++;
        time = 180;
        orchestra.play();
      }
      else if(count == 2){
        orchestra.loadMusic("Giorno.chrp");
        count++;
        time = 285;
        orchestra.play();
      }
      else if(count == 3){
        orchestra.loadMusic("Duel.chrp");
        count++;
        time = 250;
        orchestra.play();
      }
      else if(count == 4){
        orchestra.loadMusic("Imperial.chrp");
        count++;
        time = 106;
        orchestra.play();
      }
      else if(count == 5){
        orchestra.loadMusic("Megalovania.chrp");
        count++;
        time = 38;
        orchestra.play();
      }
      else if(count == 6){
        orchestra.loadMusic("Mountain.chrp");
        count++;
        time = 151;
        orchestra.play();
      }
      else if(count == 7){
        orchestra.loadMusic("Pomp.chrp");
        count++;
        time = 107;
        orchestra.play();
      }
      else if(count == 8){
        orchestra.loadMusic("US.chrp");
        count++;
        time = 219;
        orchestra.play();
      }
      else if(count == 9){
        orchestra.loadMusic("76.chrp");
        count++;
        time = 200;
        orchestra.play();
      }
      if(count == 10){
        orchestra.loadMusic("Elves.chrp");
        time = 42;
        orchestra.play();
      }
      if(count == 10){
        count = 0;
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
