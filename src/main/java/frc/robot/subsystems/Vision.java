// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private final NetworkTable table;
    private final NetworkTableEntry tx, ty, ta, tv;
    private final PhotonCamera frontCamera, backCamera;

  /** Creates a new Vision. */
  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    frontCamera = new PhotonCamera(Constants.Vision.FRONT_CAMERA_NAME);
    backCamera = new PhotonCamera(Constants.Vision.BACK_CAMERA_NAME);
  }

  public double getAprilTagRotationSpeed(){
    if(Robot.getFront()){
      var result = frontCamera.getLatestResult();
      if(result.hasTargets()){
        return result.getBestTarget().getYaw() * 0.8 / 40.0;
      }
      else{
        return getPoseRotationSpeed();
      }
    }
    else{
      var result = backCamera.getLatestResult();
      if(result.hasTargets()){
        return result.getBestTarget().getYaw() * 0.8 / 40.0;
      }
      else{
        return getPoseRotationSpeed();
      }
    }
  }

  public double getNoteRotationSpeed(){
    return getXAngle() * 0.8 / 40.0;
  }

  private double getPoseRotationSpeed(){
    return 0; //FIXME
  }

  public boolean hasValidTarget(){
    return (tv.getInteger(0) == 0) ? false : true;
  }

  public double getXAngle(){
    return tx.getDouble(0);
  }

  public double getYAngle(){
    return ty.getDouble(0);
  }

  public double getTargetArea(){
    return ta.getDouble(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
