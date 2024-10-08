package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Swerve.MAX_SPEED * 0.1).withRotationalDeadband(Constants.Swerve.MAX_ANGULAR_RATE * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    /* Controllers */
    private final Joystick translationController = new Joystick(Constants.Controllers.TRANSLATION_CONTROLLER.getPort());
    private final Joystick rotationController = new Joystick(Constants.Controllers.ROTATION_CONTROLLER.getPort());
    private final XboxController xboxController = new XboxController(Constants.Controllers.XBOX_CONTROLLER.getPort());
    private final Joystick buttons = new Joystick(Constants.Controllers.BUTTONS.getPort());

    /* Drive Controls */
    private final int translationAxis = Constants.Controllers.TRANSLATION_AXIS;
    private final int strafeAxis = Constants.Controllers.STRAFE_AXIS;
    private final int rotationAxis = Constants.Controllers.ROTATION_AXIS;
    private boolean robotRelative = Constants.ROBOT_RELATIVE;

    /* Driver Buttons */
    private final JoystickButton translationButton = new JoystickButton(translationController, Constants.Controllers.TRANSLATION_BUTTON);
    private final JoystickButton rotationButton = new JoystickButton(rotationController, Constants.Controllers.ROTATION_BUTTON);
    private final JoystickButton button7 = new JoystickButton(buttons, 7);
    private final JoystickButton button8 = new JoystickButton(buttons, 8);
    private final JoystickButton leftStick = new JoystickButton(xboxController, Constants.Controllers.XBOX_CONTROLLER_LEFT_STICK_BUTTON);
    private final JoystickButton rightStick = new JoystickButton(xboxController, Constants.Controllers.XBOX_CONTROLLER_RIGHT_STICK_BUTTON);
    private final POVButton left = new POVButton(xboxController, Constants.Controllers.LEFT);
    private final POVButton right = new POVButton(xboxController, Constants.Controllers.RIGHT);

    /* Subsystems */
    public final Swerve s_Swerve = TunerConstants.DriveTrain;
    public final LED s_LED = new LED();
    public final Vision s_Vision = new Vision(s_Swerve);
    public final Shooter s_Shooter = new Shooter();
    public final Intake s_Intake = new Intake();
    public final Elevator s_Elevator = new Elevator();

    /* Commands */
    public final Music c_Music = new Music(s_Swerve);
    private final Align c_Align = new Align(s_Swerve, s_Vision, () -> translationController.getRawAxis(translationAxis), () -> translationController.getRawAxis(strafeAxis), () -> rotationController.getRawAxis(rotationAxis), this::robotRelative);
    public final RunNote c_RunNote = new RunNote(s_Intake, s_Shooter, xboxController);
    public final AutoRunNote c_AutoRunNote = new AutoRunNote(s_Intake, s_Shooter);
    private final RunElevator c_RunElevator = new RunElevator(s_Elevator, xboxController);

    /* Telemetry */
    private final Telemetry logger = new Telemetry(Constants.Swerve.MAX_SPEED);

    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("Shoot", new InstantCommand(() -> s_Shooter.setAutoFire(true)));
        NamedCommands.registerCommand("Intake", new InstantCommand(() -> s_Intake.setAutoIntake(true)));
        NamedCommands.registerCommand("Vision", new InstantCommand(() -> Vision.setAddVisionMeasurement(true)));
        NamedCommands.registerCommand("Subwoofer", new InstantCommand(() -> s_Shooter.setSubShooting()));
        NamedCommands.registerCommand("Auto", new InstantCommand(() -> s_Shooter.setAutoShooting()));
        NamedCommands.registerCommand("Force Fire", new InstantCommand(() -> s_Shooter.setForceFire(true)));
        s_Elevator.setDefaultCommand(c_RunElevator);
        autoChooser = AutoBuilder.buildAutoChooser("DO NOTHING");
        Shuffleboard.getTab("Auto").add("Auto", autoChooser).withPosition(0, 0).withSize(2, 1);
        Shuffleboard.selectTab("Auto");
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        if(Constants.USE_XBOX_CONTROLLER) {
            s_Swerve.setDefaultCommand(
            s_Swerve.applyRequest(() -> drive.withVelocityX(-xboxController.getRawAxis(translationAxis) * Constants.Swerve.MAX_SPEED) 
                                             .withVelocityY(-xboxController.getRawAxis(strafeAxis) * Constants.Swerve.MAX_SPEED) 
                                             .withRotationalRate(xboxController.getRawAxis(rotationAxis) * Constants.Swerve.MAX_ANGULAR_RATE)));
        }
        else {
            s_Swerve.setDefaultCommand(
            s_Swerve.applyRequest(() -> drive.withVelocityX(translationController.getRawAxis(translationAxis) * Constants.Swerve.MAX_SPEED) 
                                             .withVelocityY(translationController.getRawAxis(strafeAxis) * Constants.Swerve.MAX_SPEED) 
                                             .withRotationalRate(rotationController.getRawAxis(rotationAxis) * Constants.Swerve.MAX_ANGULAR_RATE) 
            ));
        }
        

        /* Driver Buttons */
        translationButton.onTrue(new InstantCommand(() -> s_Swerve.getPigeon2().setYaw(0)));
        rotationButton.whileTrue(c_Align);
        button7.whileTrue(new InstantCommand(() -> robotRelative = true)).onFalse(new InstantCommand(() -> robotRelative = false));
        button8.whileTrue(c_Align);
        leftStick.onTrue(new InstantCommand(() -> s_Shooter.setLongShooting()));
        rightStick.onTrue(new InstantCommand(() -> s_Shooter.setAutoShooting()));
        leftStick.and(rightStick).onTrue(new InstantCommand(() -> s_Shooter.setPoopShooting()));
        right.onTrue(new InstantCommand(() -> s_Shooter.setSubShooting()));
        left.onTrue(new InstantCommand(() -> s_Shooter.setPodiumShooting()));

        s_Swerve.registerTelemetry(logger::telemeterize);
    }

    public boolean robotRelative(){
        return robotRelative;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Logger.recordOutput("Auto", autoChooser.getSelected().getName());
        return autoChooser.getSelected();
    }
}
