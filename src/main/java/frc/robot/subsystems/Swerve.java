package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public final SwerveModule[] mSwerveMods;
    private final Pigeon2 gyro;
    private final GenericEntry mod0Cancoder, mod1Cancoder, mod2Cancoder, mod3Cancoder, yaw, poseX, poseY;
    private final ComplexWidget fieldWidget;
    private final Field2d field = new Field2d(); 
    private final SwerveDrivePoseEstimator poseEstimator;
    private boolean noteTracking;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.PIGEON_ID, Constants.CANIVORE_BUS);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);


        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.CONSTANTS),
            new SwerveModule(1, Constants.Swerve.Mod1.CONSTANTS),
            new SwerveModule(2, Constants.Swerve.Mod2.CONSTANTS),
            new SwerveModule(3, Constants.Swerve.Mod3.CONSTANTS)
        };

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.SWERVE_KINEMATICS, getGyroYaw(), getModulePositions(), new Pose2d(), Constants.Swerve.STATE_STDS, Constants.Vision.TAG_VISION_STDS);

        mod0Cancoder = Shuffleboard.getTab("Drive").add("Mod 0 Cancoder", mSwerveMods[0].getState().angle.getDegrees()).withPosition(0, 0).getEntry();
        mod1Cancoder = Shuffleboard.getTab("Drive").add("Mod 1 Cancoder", mSwerveMods[1].getState().angle.getDegrees()).withPosition(1, 0).getEntry();
        mod2Cancoder = Shuffleboard.getTab("Drive").add("Mod 2 Cancoder", mSwerveMods[2].getState().angle.getDegrees()).withPosition(2, 0).getEntry();
        mod3Cancoder = Shuffleboard.getTab("Drive").add("Mod 3 Cancoder", mSwerveMods[3].getState().angle.getDegrees()).withPosition(3, 0).getEntry();
        yaw = Shuffleboard.getTab("Drive").add("Yaw", getHeading().getDegrees()).withPosition(0, 1).getEntry();
        poseX = Shuffleboard.getTab("Drive").add("X", 0).withPosition(0, 2).getEntry();
        poseY = Shuffleboard.getTab("Drive").add("Y", 0).withPosition(1, 2).getEntry();
        fieldWidget = Shuffleboard.getTab("Drive").add("Field", field).withPosition(4, 0).withSize(4, 3);
        fieldWidget.toString();
        configPathPlanner();
    }

    public boolean getNoteTracking(){
        return noteTracking;
    }

    public void setNoteTracking(boolean noteTracking){
        this.noteTracking = noteTracking;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void zeroHeading(){
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
        gyro.setYaw(0);
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getSpeed(){
        return Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public void configPathPlanner(){
        AutoBuilder.configureHolonomic(this::getPose, this::setPose, this::getSpeed, (speeds) -> setModuleStates(Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(speeds)),
            new HolonomicPathFollowerConfig(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0), Constants.Swerve.MAX_SPEED, Constants.Swerve.DRIVE_BASE_RADIUS, new ReplanningConfig(false, false)),
            () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()){
                    if(alliance.get() == DriverStation.Alliance.Red){
                        return true;
                    }
                    else{
                        return false;
                    }
                }
                else{
                    return false;
                }
            },
            this);
    }

    public void updatePoseVision(EstimatedRobotPose pose){
        poseEstimator.addVisionMeasurement(new Pose2d(new Translation2d(pose.estimatedPose.getX(), pose.estimatedPose.getY()), getGyroYaw()), pose.timestampSeconds);
    }

    @Override
    public void periodic(){
        poseEstimator.update(getGyroYaw(), getModulePositions());

        if(Robot.robotContainer.s_Intake.botFull()){
            setNoteTracking(false);
        }
        else{
            setNoteTracking(true);
        }

        field.setRobotPose(getPose());
        poseX.setDouble(getPose().getX());
        poseY.setDouble(getPose().getY());
       
        mod0Cancoder.setDouble(mSwerveMods[0].getCANcoder().getDegrees());
        mod1Cancoder.setDouble(mSwerveMods[1].getCANcoder().getDegrees());
        mod2Cancoder.setDouble(mSwerveMods[2].getCANcoder().getDegrees());
        mod3Cancoder.setDouble(mSwerveMods[3].getCANcoder().getDegrees());
        
        yaw.setDouble(gyro.getYaw().getValue());

        Logger.recordOutput("Yaw", getGyroYaw());
        Logger.recordOutput("Swerve States", getModuleStates());
        Logger.recordOutput("Pose", getPose());
    }
}