// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private final PhotonCamera frontCamera, backCamera, noteCamera;
    private AprilTagFieldLayout atfl;
    private final PhotonPoseEstimator frontPoseEstimator, backPoseEstimator;
    private final Swerve s_Swerve;
    private double targetX, targetY, robotX, robotY;
    private final GenericEntry distanceEntry;
    private double robotRelVelocityVector, fieldRelVelocityX, fieldRelVelocityY;
    private double xFlyAngle, yFlyAngle, originalAngle, xFlyWrist, yFlyWrist, velocityAngleOffset;
    private double velocityCompAngle = 0.04;
    private double velocityCompWrist = 0.01;


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

    SmartDashboard.putNumber("Comp Angle", velocityCompAngle);
    SmartDashboard.putNumber("Comp Wrist", velocityCompWrist);
  }

  public Optional<Rotation2d> getRotationTargetOverride(){
    if(Robot.robotContainer.s_Intake.botFull()){
      return Optional.of(PhotonUtils.getYawToPose(new Pose2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY(), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(0))));
    }
    else{
      return Optional.empty();
    }
  }
  
  public double getAprilTagRotationSpeed(){
    if(frontCamera.getLatestResult().hasTargets() && Robot.getFront() || backCamera.getLatestResult().hasTargets() && !Robot.getFront()){
      if(Robot.getFront()){
        var result = frontCamera.getLatestResult();
        if(result.hasTargets() && !Robot.getRedAlliance()){
          xFlyAngle = (robotX - targetX) - (fieldRelVelocityX * velocityCompAngle);
          yFlyAngle = (robotY - targetY) - (fieldRelVelocityY * velocityCompAngle);
          originalAngle = Math.toDegrees(Math.atan2(robotY - targetY, robotX - targetX));

          velocityAngleOffset = Math.toDegrees(Math.atan2(yFlyAngle, xFlyAngle)) - originalAngle;

          return velocityAngleOffset - result.getTargets().get(result.getTargets().size() == 1 ? 0 : 1).getYaw() * 0.8 / 9.0;
          //return -result.getTargets().get(0).getYaw() * 0.8 / 9.0;
        }
        else if(result.hasTargets() && Robot.getRedAlliance()){
          xFlyAngle = (targetX - robotX) - (fieldRelVelocityX * velocityCompAngle);
          yFlyAngle = (targetY - robotY) - (fieldRelVelocityY * velocityCompAngle);
          originalAngle = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX));
      
          velocityAngleOffset = Math.toDegrees(Math.atan2(yFlyAngle, xFlyAngle)) - originalAngle;
          
          return velocityAngleOffset - result.getTargets().get(result.getTargets().size() == 1 ? 0 : 1).getYaw() * 0.8 / 9.0;
          //return -result.getTargets().get(result.getTargets().size() == 1 ? 0 : 1).getYaw() * 0.8 / 9.0;
        }
        else{
          return 0;
        }
      }
      else{
        var result = backCamera.getLatestResult();
        if(result.hasTargets() && !Robot.getRedAlliance()){
          xFlyAngle = (robotX - targetX) - (fieldRelVelocityX * velocityCompAngle);
          yFlyAngle = (robotY - targetY) - (fieldRelVelocityY * velocityCompAngle);
          originalAngle = Math.toDegrees(Math.atan2(robotY - targetY, robotX - targetX));

          velocityAngleOffset = Math.toDegrees(Math.atan2(yFlyAngle, xFlyAngle)) - originalAngle;

          return velocityAngleOffset - result.getTargets().get(result.getTargets().size() == 1 ? 0 : 1).getYaw() * 0.8 / 9.0;
          //return -result.getTargets().get(0).getYaw() * 0.8 / 9.0;
        }
        else if(result.hasTargets() && Robot.getRedAlliance()){
          xFlyAngle = (targetX - robotX) - (fieldRelVelocityX * velocityCompAngle);
          yFlyAngle = (targetY - robotY) - (fieldRelVelocityY * velocityCompAngle);
          originalAngle = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX));
      
          velocityAngleOffset = Math.toDegrees(Math.atan2(yFlyAngle, xFlyAngle)) - originalAngle;
          
          return velocityAngleOffset - result.getTargets().get(result.getTargets().size() == 1 ? 0 : 1).getYaw() * 0.8 / 9.0;
          //return -result.getTargets().get(result.getTargets().size() == 1 ? 0 : 1).getYaw() * 0.8 / 9.0;
        }
        else{
          return 0;
        }
      }
    }
    else{
      if(Robot.getFront()){
        return (PhotonUtils.getYawToPose(s_Swerve.getPose(), new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(0))).rotateBy(Rotation2d.fromDegrees(180)).getDegrees()) * 8.0 / 42.0;
      }
      else{
        return PhotonUtils.getYawToPose(s_Swerve.getPose(), new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(0))).getDegrees() * 8.0 / 42.0;
      }
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
    //return Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2));
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
  
  public double getNoteLatency(){
    return noteCamera.getLatestResult().getLatencyMillis();
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
    if(frontCamera.getLatestResult().hasTargets()){
      var pose = getFrontEstimatedGlobalPose();
      if(pose.isPresent()){
        s_Swerve.updatePoseVision(pose.get());
      }
    }
    else if(backCamera.getLatestResult().hasTargets()){
      var pose = getBackEstimatedGlobalPose();
      if(pose.isPresent()){
        s_Swerve.updatePoseVision(pose.get());
      }
    }

    robotRelVelocityVector = Math.sqrt(Math.pow(s_Swerve.getSpeed().vxMetersPerSecond, 2) + Math.pow(s_Swerve.getSpeed().vyMetersPerSecond, 2));
    fieldRelVelocityX = robotRelVelocityVector * Math.cos(s_Swerve.getGyroYaw().getRadians() + Math.PI) * Math.signum(s_Swerve.getSpeed().vxMetersPerSecond);
    fieldRelVelocityY = robotRelVelocityVector * Math.sin(s_Swerve.getGyroYaw().getRadians() + Math.PI) * Math.signum(s_Swerve.getSpeed().vyMetersPerSecond);

    velocityCompAngle = SmartDashboard.getNumber("Comp Angle", velocityCompAngle);
    velocityCompWrist = SmartDashboard.getNumber("Comp Wrist", velocityCompWrist);

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
  }
}
