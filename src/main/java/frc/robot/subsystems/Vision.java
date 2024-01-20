// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private final NetworkTable table;
    private final NetworkTableEntry tx, ty, ta, tv;
    private final PhotonCamera frontCamera, backCamera;
    private AprilTagFieldLayout atfl;
    private final PhotonPoseEstimator frontPoseEstimator, backPoseEstimator;
    private final Swerve s_Swerve;
    private double targetX, targetY;

  /** Creates a new Vision. */
  public Vision(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    frontCamera = new PhotonCamera(Constants.Vision.FRONT_CAMERA_NAME);
    backCamera = new PhotonCamera(Constants.Vision.BACK_CAMERA_NAME);

    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {}

    frontPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera, Constants.Vision.FRONT_CAMERA_TRANSFORM);
    backPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamera, Constants.Vision.BACK_CAMERA_TRANSFORM);
  }

  public double getAprilTagRotationSpeed(){
    if(Robot.getFront()){
      var result = frontCamera.getLatestResult();
      if(result.hasTargets()){
        return -result.getBestTarget().getYaw() * 0.8 / 30.0;
      }
      else{
        return getPoseRotationSpeed();
      }
    }
    else{
      var result = backCamera.getLatestResult();
      if(result.hasTargets()){
        return -result.getBestTarget().getYaw() * 0.8 / 30.0;
      }
      else{
        return getPoseRotationSpeed();
      }
    }
  }

  public double getNoteRotationSpeed(){
    return -getXAngle() * 0.8 / 30.0;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(){
    if(backCamera.getLatestResult().hasTargets()){
      return backPoseEstimator.update();
    }
    else{
      return frontPoseEstimator.update();
    }
  }

  private double getPoseRotationSpeed(){
    return -(s_Swerve.getGyroYaw().getRadians() - Math.atan((targetY - s_Swerve.getPose().getY()) / (targetX - s_Swerve.getPose().getX()))) * 0.8 / 30.0;
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
    s_Swerve.updatePoseVision(getEstimatedGlobalPose());
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      if(alliance.get() == DriverStation.Alliance.Blue){
        targetX = Constants.Vision.TARGET_X;
        targetY = Constants.Vision.TARGET_Y_BLUE;
      }
      else{
        targetX = Constants.Vision.TARGET_X;
        targetY = Constants.Vision.TARGET_Y_RED;
      }
    }
  }
}
