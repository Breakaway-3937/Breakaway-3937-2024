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
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private final PhotonCamera frontCamera, noteCamera;//, backCamera;
    private AprilTagFieldLayout atfl;
    private final PhotonPoseEstimator frontPoseEstimator;//, backPoseEstimator;
    private final Swerve s_Swerve;
    private double targetX, targetY;
    private boolean blue = false;

  /** Creates a new Vision. */
  public Vision(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;

    frontCamera = new PhotonCamera(Constants.Vision.FRONT_CAMERA_NAME);
    //backCamera = new PhotonCamera(Constants.Vision.BACK_CAMERA_NAME);
    noteCamera = new PhotonCamera(Constants.Vision.NOTE_CAMERA_NAME);

    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    }
    catch(IOException e){}

    frontPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera, Constants.Vision.FRONT_CAMERA_TRANSFORM);
    //backPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamera, Constants.Vision.BACK_CAMERA_TRANSFORM);

    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
  }

  public Optional<Rotation2d> getRotationTargetOverride(){
    var result = frontCamera.getLatestResult();
    if(result.hasTargets()){
        return Optional.of(getAprilTagRotation2d(result));
    }
    else{
        return Optional.empty();
    }
  }

  public Rotation2d getAprilTagRotation2d(PhotonPipelineResult result){
    return new Rotation2d((s_Swerve.getHeading().getDegrees() + result.getBestTarget().getYaw()) * Math.PI / 180.0);
  }

  public double getAprilTagRotationSpeed(){
    if(Robot.getFront()){
      var result = frontCamera.getLatestResult();
      if(result.hasTargets() && blue){
        return -result.getTargets().get(0).getYaw() * 0.8 / 15.0;
      }
      else if(result.hasTargets() && !blue){
        return -result.getTargets().get(1).getYaw() * 0.8 / 15.0;
      }
      else{
        return getPoseRotationSpeed();
      }
    }
    else{
      return 0;
      /*var result = backCamera.getLatestResult();
      if(result.hasTargets()){
        return -result.getBestTarget().getYaw() * 0.8 / 15.0;
      }
      else{
        return getPoseRotationSpeed();
      }*/
    }
  }

  public double getNoteRotationSpeed(){
    var result = noteCamera.getLatestResult();
    if(result.hasTargets()){
      return -result.getBestTarget().getYaw() * 0.8 / 15.0;
    }
    else{
      return 0;
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(){
    if(false){//backCamera.getLatestResult().hasTargets()){
      return Optional.empty();//backPoseEstimator.update();
    }
    else{
      return frontPoseEstimator.update();
    }
  }

  private double getPoseRotationSpeed(){
    return -(s_Swerve.getGyroYaw().getDegrees() - Math.toDegrees(Math.atan((targetY - s_Swerve.getPose().getY()) / (targetX - s_Swerve.getPose().getX())))) * 0.8 / 15.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    s_Swerve.updatePoseVision(getEstimatedGlobalPose());
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      if(alliance.get() == DriverStation.Alliance.Blue){
        blue = true;
        targetX = Constants.Vision.TARGET_X_BLUE;
        targetY = Constants.Vision.TARGET_Y_BLUE;
      }
      else{
        blue = false;
        targetX = Constants.Vision.TARGET_X_RED;
        targetY = Constants.Vision.TARGET_Y_RED;
      }
    }
  }
}
