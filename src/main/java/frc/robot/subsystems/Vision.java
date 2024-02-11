// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private final PhotonCamera frontCamera, backCamera, noteCamera;
    private AprilTagFieldLayout atfl;
    private final PhotonPoseEstimator frontPoseEstimator, backPoseEstimator;
    private final Swerve s_Swerve;
    private double targetX, targetY;
    private boolean blue;
    private final PIDController posePid = new PIDController(0.3, 0, 0);

  /** Creates a new Vision. */
  public Vision(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;

    frontCamera = new PhotonCamera(Constants.Vision.FRONT_CAMERA_NAME);
    backCamera = new PhotonCamera(Constants.Vision.BACK_CAMERA_NAME);
    noteCamera = new PhotonCamera(Constants.Vision.NOTE_CAMERA_NAME);

    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    }
    catch(IOException e){}

    frontPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera, Constants.Vision.FRONT_CAMERA_TRANSFORM);
    backPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamera, Constants.Vision.BACK_CAMERA_TRANSFORM);
    frontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    backPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
  }

  public Optional<Rotation2d> getRotationTargetOverride(){
    if(Robot.robotContainer.s_Intake.botFull()){
        return Optional.of(PhotonUtils.getYawToPose(s_Swerve.getPose(), new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromRadians(0))));
    }
    else{
        return Optional.empty();
    }
  }
  
  public double getAprilTagRotationSpeed(){
    return posePid.calculate(PhotonUtils.getYawToPose(s_Swerve.getPose(), new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromRadians(0))).getRadians());
  }

  public double getNoteRotationSpeed(){
    var result = noteCamera.getLatestResult();
    if(result.hasTargets()){
      return -result.getBestTarget().getYaw() * 0.8 / 9.0;
    }
    else{
      return 0;
    }
  }

  public boolean getNoteTargets(){
    return noteCamera.getLatestResult().hasTargets();
  }

  public Optional<EstimatedRobotPose> getFrontEstimatedGlobalPose(){
    return frontPoseEstimator.update();
  }

  public Optional<EstimatedRobotPose> getBackEstimatedGlobalPose(){
    return backPoseEstimator.update();
  }

  public boolean isDead(){
    if(!frontCamera.isConnected() && !backCamera.isConnected()){
      return true;
    }
    else{
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    s_Swerve.updatePoseVision(getFrontEstimatedGlobalPose(), blue);
    s_Swerve.updatePoseVision(getBackEstimatedGlobalPose(), blue);
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
