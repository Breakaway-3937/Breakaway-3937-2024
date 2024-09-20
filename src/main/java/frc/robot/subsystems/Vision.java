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
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private final PhotonCamera frontCamera, backCamera, noteCamera;
    private AprilTagFieldLayout atfl;
    private final PhotonPoseEstimator frontPoseEstimator, backPoseEstimator;
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;
    private final Swerve s_Swerve;
    private double targetX, targetY, robotX, robotY;
    private final GenericEntry distanceEntry, targetID, compAngleEntry, compWristEntry;
    private double fieldRelVelocityX, fieldRelVelocityY;
    private double xFlyAngle, yFlyAngle, originalAngle, xFlyWrist, yFlyWrist, velocityAngleOffset;
    private double velocityCompAngle = 0.05;
    private double velocityCompWrist = 0.4;
    private PhotonTrackedTarget target;
    private boolean frontPoseBad, backPoseBad = false;
    private static boolean addVisionMeasurement = false;
    private double ambiguity, targetYaw = Double.POSITIVE_INFINITY;
    private Pose3d[] usedTagsFront;
    private Pose3d[] usedTagsBack;


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
  
    if(Robot.isSimulation()) {
       // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(atfl);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(frontCamera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, Constants.Vision.FRONT_CAMERA_TRANSFORM);

            cameraSim.enableDrawWireframe(true);
    }
  
  }

  public Optional<Rotation2d> getRotationTargetOverride(){
    if(Robot.robotContainer.s_Intake.botFull()){
      return Optional.of(PhotonUtils.getYawToPose(new Pose2d(s_Swerve.getState().Pose.getX(), s_Swerve.getState().Pose.getX(), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(0))));
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
          return PhotonUtils.getYawToPose(s_Swerve.getState().Pose, new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(0))).rotateBy(Rotation2d.fromDegrees(180)).getDegrees() * 8.0 / 42.0;
        }
        else{
          return PhotonUtils.getYawToPose(s_Swerve.getState().Pose, new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(0))).getDegrees() * 8.0 / 42.0;
        }
      }
      else{
        if(Robot.getFront()){
          return PhotonUtils.getYawToPose(s_Swerve.getState().Pose, new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(0))).getDegrees() * 8.0 / 42.0;
        }
        else{
          return PhotonUtils.getYawToPose(s_Swerve.getState().Pose, new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(0))).rotateBy(Rotation2d.fromDegrees(180)).getDegrees() * 8.0 / 42.0;
        }
      }
    }
  }

  public double getDistance(){
    robotX = s_Swerve.getState().Pose.getX();
    robotY = s_Swerve.getState().Pose.getY();
          
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

  public static void setAddVisionMeasurement(boolean newAddVisionMeasurement){
    addVisionMeasurement = newAddVisionMeasurement;
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

  public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

  @Override
  public void periodic() {
    if(Robot.isSimulation()) {
      var debugField = getSimDebugField();
      simulationPeriodic(s_Swerve.getState().Pose);
      debugField.getObject("EstimatedRobot").setPose(s_Swerve.getState().Pose);
    }

    // This method will be called once per scheduler run
    var frontResult = frontCamera.getLatestResult();
    if(frontResult.hasTargets()){
      var pose = getFrontEstimatedGlobalPose();
      if(pose.isPresent()){
        for(int i = 0; i < pose.get().targetsUsed.size(); i++){
          if(Math.abs(pose.get().targetsUsed.get(i).getBestCameraToTarget().getTranslation().getX()) > 6.0){
            frontPoseBad = true;
          }
        }
        if(!frontPoseBad && frontResult.getBestTarget().getPoseAmbiguity() < 0.2 && frontResult.getBestTarget().getPoseAmbiguity() >= 0){
          ambiguity = frontResult.getBestTarget().getPoseAmbiguity();
          if(!DriverStation.isAutonomousEnabled() || addVisionMeasurement) {
            s_Swerve.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds, Constants.Vision.TAG_VISION_STDS_FRONT);
          }
        }

        for(int i = 0; i < pose.get().targetsUsed.size(); i++){
          usedTagsFront[i] = frontPoseEstimator.getFieldTags().getTagPose(pose.get().targetsUsed.get(i).getFiducialId()).get();
        }
        
        Logger.recordOutput("Front Cam Used Tags", (usedTagsFront.length == 0) ? new Pose3d[]{new Pose3d(-1, -1, -1, new Rotation3d())} : usedTagsFront);
      }
    }

    var backResult = backCamera.getLatestResult();
    if(backResult.hasTargets() && DriverStation.isTeleopEnabled()){
      var pose = getBackEstimatedGlobalPose();
      if(pose.isPresent()){
        for(int i = 0; i < pose.get().targetsUsed.size(); i++){
          if(Math.abs(pose.get().targetsUsed.get(i).getBestCameraToTarget().getTranslation().getX()) > 6.0){
            backPoseBad = true;
          }
        }
        if(!backPoseBad && backResult.getBestTarget().getPoseAmbiguity() < 0.2 && backResult.getBestTarget().getPoseAmbiguity() >= 0 && backResult.getBestTarget().getPoseAmbiguity() < ambiguity){
          if(!DriverStation.isAutonomousEnabled() || addVisionMeasurement){
            s_Swerve.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds, Constants.Vision.TAG_VISION_STDS_BACK);
          }
        }

        for(int i = 0; i < pose.get().targetsUsed.size(); i++){
          usedTagsBack[i] = backPoseEstimator.getFieldTags().getTagPose(pose.get().targetsUsed.get(i).getFiducialId()).get();
        }

        Logger.recordOutput("Back Cam Used Tags", (usedTagsBack.length == 0) ? new Pose3d[]{new Pose3d(-1, -1, -1, new Rotation3d())} : usedTagsBack);
      }
    }

    Logger.recordOutput("Front Pose Bad", frontPoseBad);
    Logger.recordOutput("Back Pose Bad", backPoseBad);

    if(target != null){
      Logger.recordOutput("Target ID", target.getFiducialId());
      targetID.setDouble(target.getFiducialId());
    }

    if(Robot.getRedAlliance() && DriverStation.isAutonomousEnabled()){
      fieldRelVelocityX = ChassisSpeeds.fromRobotRelativeSpeeds(s_Swerve.getState().speeds, Rotation2d.fromDegrees(s_Swerve.getHeading())).vxMetersPerSecond;
      fieldRelVelocityY = ChassisSpeeds.fromRobotRelativeSpeeds(s_Swerve.getState().speeds, Rotation2d.fromDegrees(s_Swerve.getHeading())).vyMetersPerSecond;
    }
    else{
      fieldRelVelocityX = ChassisSpeeds.fromRobotRelativeSpeeds(s_Swerve.getState().speeds, Rotation2d.fromDegrees(s_Swerve.getHeading() + 180)).vxMetersPerSecond;
      fieldRelVelocityY = ChassisSpeeds.fromRobotRelativeSpeeds(s_Swerve.getState().speeds, Rotation2d.fromDegrees(s_Swerve.getHeading() + 180)).vyMetersPerSecond;
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
