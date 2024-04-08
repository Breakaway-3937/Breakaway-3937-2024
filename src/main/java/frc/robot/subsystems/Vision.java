// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private final PhotonCamera frontCamera, backCamera, noteCamera;
    private AprilTagFieldLayout atfl;
    private final PhotonPoseEstimator frontPoseEstimator, backPoseEstimator;
    private final Swerve s_Swerve;
    private double targetX, targetY, robotX, robotY;
    private final GenericEntry distanceEntry, targetID, compAngleEntry, compWristEntry;
    private double fieldRelVelocityX, fieldRelVelocityY;
    private double xFlyAngle, yFlyAngle, originalAngle, xFlyWrist, yFlyWrist, velocityAngleOffset;
    private double velocityCompAngle = 0.05;
    private double velocityCompWrist = 0.425;
    private PhotonTrackedTarget target;
    private boolean frontPoseBad, backPoseBad = false;
    private double ambiguity, targetYaw = Double.POSITIVE_INFINITY;


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

    distanceEntry = Shuffleboard.getTab("Shooter").add("Distance", 0).withPosition(2, 0).getEntry();
    targetID = Shuffleboard.getTab("Shooter").add("Target ID", 0).withPosition(3, 0).getEntry();

    compAngleEntry = Shuffleboard.getTab("Shooter").add("Comp Angle Offset", velocityCompAngle).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).withPosition(0, 2).getEntry();
    compWristEntry = Shuffleboard.getTab("Shooter").add("Comp Wrist Offset", velocityCompWrist).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).withPosition(2, 2).getEntry();
  }

  public Optional<Rotation2d> getRotationTargetOverride(){
    if(Robot.robotContainer.s_Intake.botFull()){
      return Optional.of(PhotonUtils.getYawToPose(new Pose2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY(), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(0))));
    }
    else{
      return Optional.empty();
    }
  }

  public double getAprilTagYaw(){
    return targetYaw;
  }
  
  public double getAprilTagRotationSpeed(){
    if(frontCamera.getLatestResult().hasTargets() && Robot.getFront() || backCamera.getLatestResult().hasTargets() && !Robot.getFront()){
      if(Robot.getFront()){
        var result = frontCamera.getLatestResult();
        if(result.hasTargets() && !Robot.getRedAlliance()){
          for(int i = 0; i < result.getTargets().size(); i++){
            if(result.getTargets().get(i).getFiducialId() == 7){
              target = result.getTargets().get(i);
              break;
            }
          }
          xFlyAngle = (robotX - targetX) - (fieldRelVelocityX * velocityCompAngle);
          yFlyAngle = (robotY - targetY) - (fieldRelVelocityY * velocityCompAngle);
          originalAngle = Math.toDegrees(Math.atan2(robotY - targetY, robotX - targetX));

          velocityAngleOffset = Math.toDegrees(Math.atan2(yFlyAngle, xFlyAngle)) - originalAngle;

          if(target != null){
            targetYaw = target.getYaw();
            return velocityAngleOffset - target.getYaw() * 0.8 / 9.0;
          }
          else{
            return 0;
          }
        }
        else if(result.hasTargets() && Robot.getRedAlliance()){
          for(int i = 0; i < result.getTargets().size(); i++){
            if(result.getTargets().get(i).getFiducialId() == 4){
              target = result.getTargets().get(i);
              break;
            }
          }
          xFlyAngle = (targetX - robotX) - (fieldRelVelocityX * velocityCompAngle);
          yFlyAngle = (targetY - robotY) - (fieldRelVelocityY * velocityCompAngle);
          originalAngle = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX));
      
          velocityAngleOffset = Math.toDegrees(Math.atan2(yFlyAngle, xFlyAngle)) - originalAngle;
          
          if(target != null){
            targetYaw = target.getYaw();
            return velocityAngleOffset - target.getYaw() * 0.8 / 9.0;
          }
          else{
            return 0;
          }
        }
        else{
          return 0;
        }
      }
      else{
        var result = backCamera.getLatestResult();
        if(result.hasTargets() && !Robot.getRedAlliance()){
          for(int i = 0; i < result.getTargets().size(); i++){
            if(result.getTargets().get(i).getFiducialId() == 7){
              target = result.getTargets().get(i);
              break;
            }
          }
          xFlyAngle = (robotX - targetX) - (fieldRelVelocityX * velocityCompAngle);
          yFlyAngle = (robotY - targetY) - (fieldRelVelocityY * velocityCompAngle);
          originalAngle = Math.toDegrees(Math.atan2(robotY - targetY, robotX - targetX));

          velocityAngleOffset = Math.toDegrees(Math.atan2(yFlyAngle, xFlyAngle)) - originalAngle;

          if(target != null){
            targetYaw = target.getYaw();
            return velocityAngleOffset - target.getYaw() * 0.8 / 9.0;
          }
          else{
            return 0;
          }
        }
        else if(result.hasTargets() && Robot.getRedAlliance()){
          for(int i = 0; i < result.getTargets().size(); i++){
            if(result.getTargets().get(i).getFiducialId() == 4){
              target = result.getTargets().get(i);
              break;
            }
          }
          xFlyAngle = (targetX - robotX) - (fieldRelVelocityX * velocityCompAngle);
          yFlyAngle = (targetY - robotY) - (fieldRelVelocityY * velocityCompAngle);
          originalAngle = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX));
      
          velocityAngleOffset = Math.toDegrees(Math.atan2(yFlyAngle, xFlyAngle)) - originalAngle;
          
          if(target != null){
            targetYaw = target.getYaw();
            return velocityAngleOffset - target.getYaw() * 0.8 / 9.0;
          }
          else{
            return 0;
          }
        }
        else{
          return 0;
        }
      }
    }
    else{
      if(Robot.getRedAlliance()){
        if(Robot.getFront()){
          return PhotonUtils.getYawToPose(s_Swerve.getPose(), new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(0))).rotateBy(Rotation2d.fromDegrees(180)).getDegrees() * 8.0 / 42.0;
        }
        else{
          return PhotonUtils.getYawToPose(s_Swerve.getPose(), new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(0))).getDegrees() * 8.0 / 42.0;
        }
      }
      else{
        if(Robot.getFront()){
          return PhotonUtils.getYawToPose(s_Swerve.getPose(), new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(0))).getDegrees() * 8.0 / 42.0;
        }
        else{
          return PhotonUtils.getYawToPose(s_Swerve.getPose(), new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(0))).rotateBy(Rotation2d.fromDegrees(180)).getDegrees() * 8.0 / 42.0;
        }
      }
    }
  }

  public double getAprilTagStrafeSpeed(){
    var result = frontCamera.getLatestResult();
    if(result.hasTargets()){
      return -result.getBestTarget().getYaw() * 8.0 / 148.0;
    }
    else{
      return 0;
    }
  }

  public double getDistance(){
    robotX = s_Swerve.getPose().getX();
    robotY = s_Swerve.getPose().getY();
          
    if(Robot.getRedAlliance()){
      xFlyWrist = (targetX - robotX) - (fieldRelVelocityX * velocityCompWrist);
            
      yFlyWrist = (targetY - robotY) - (fieldRelVelocityY * velocityCompWrist);
    }
    else{
      xFlyWrist = (robotX - targetX) - (fieldRelVelocityX * velocityCompWrist);
            
      yFlyWrist = (robotY - targetY) - (fieldRelVelocityY * velocityCompWrist);
    }

    return Math.sqrt(Math.pow(xFlyWrist, 2) + Math.pow(yFlyWrist, 2));
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
    var frontResult = frontCamera.getLatestResult();
    if(frontResult.hasTargets()){
      var pose = getFrontEstimatedGlobalPose();
      if(pose.isPresent()){
        for(int i = 0; i < pose.get().targetsUsed.size(); i++){
          if(Math.abs(pose.get().targetsUsed.get(i).getBestCameraToTarget().getTranslation().getX()) > 6){
            frontPoseBad = true;
          }
        }
        if(!frontPoseBad && frontResult.getBestTarget().getPoseAmbiguity() < 0.2){
          ambiguity = frontResult.getBestTarget().getPoseAmbiguity();
          s_Swerve.updatePoseVision(pose.get());
        }
        Logger.recordOutput("Front Cam Used Tag", pose.get().targetsUsed.get(0).getFiducialId());
      }
    }
    var backResult = backCamera.getLatestResult();
    if(backResult.hasTargets() && DriverStation.isTeleopEnabled()){
      var pose = getBackEstimatedGlobalPose();
      if(pose.isPresent()){
        for(int i = 0; i < pose.get().targetsUsed.size(); i++){
          if(Math.abs(pose.get().targetsUsed.get(i).getBestCameraToTarget().getTranslation().getX()) > 6){
            backPoseBad = true;
          }
        }
        if(!backPoseBad && backResult.getBestTarget().getPoseAmbiguity() < 0.2 && backResult.getBestTarget().getPoseAmbiguity() < ambiguity){
          s_Swerve.updatePoseVision(pose.get());
        }
        Logger.recordOutput("Back Cam Used Tag", pose.get().targetsUsed.get(0).getFiducialId());
      }
    }

    Logger.recordOutput("Front Pose Bad", frontPoseBad);
    Logger.recordOutput("Back Pose Bad", backPoseBad);

    if(target != null){
      Logger.recordOutput("Target ID", target.getFiducialId());
      targetID.setDouble(target.getFiducialId());
    }

    if(Robot.getRedAlliance() && DriverStation.isAutonomousEnabled()){
      fieldRelVelocityX = ChassisSpeeds.fromRobotRelativeSpeeds(s_Swerve.getSpeed(), Rotation2d.fromDegrees(s_Swerve.getGyroYaw().getDegrees())).vxMetersPerSecond;
      fieldRelVelocityY = ChassisSpeeds.fromRobotRelativeSpeeds(s_Swerve.getSpeed(), Rotation2d.fromDegrees(s_Swerve.getGyroYaw().getDegrees())).vyMetersPerSecond;
    }
    else{
      fieldRelVelocityX = ChassisSpeeds.fromRobotRelativeSpeeds(s_Swerve.getSpeed(), Rotation2d.fromDegrees(s_Swerve.getGyroYaw().getDegrees() + 180)).vxMetersPerSecond;
      fieldRelVelocityY = ChassisSpeeds.fromRobotRelativeSpeeds(s_Swerve.getSpeed(), Rotation2d.fromDegrees(s_Swerve.getGyroYaw().getDegrees() + 180)).vyMetersPerSecond;
    }

    velocityCompAngle = compAngleEntry.getDouble(velocityCompAngle);
    velocityCompWrist = compWristEntry.getDouble(velocityCompWrist);

    distanceEntry.setDouble(getDistance());
    Logger.recordOutput("Distance", getDistance());
    
    if(!Robot.getRedAlliance()){
      targetX = Constants.Vision.TARGET_X_BLUE;
      targetY = Constants.Vision.TARGET_Y_BLUE;
    }
    else{
      targetX = Constants.Vision.TARGET_X_RED;
      targetY = Constants.Vision.TARGET_Y_RED;
    }

    frontPoseBad = false;
    backPoseBad = false;
  }
}
