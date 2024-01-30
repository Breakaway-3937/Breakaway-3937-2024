package frc.robot;

import org.littletonrobotics.junction.Logger;

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

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick translationController = new Joystick(Constants.Controllers.TRANSLATION_CONTROLLER.getPort());
    private final Joystick rotationController = new Joystick(Constants.Controllers.ROTATION_CONTROLLER.getPort());
    public final XboxController xboxController = new XboxController(Constants.Controllers.XBOX_CONTROLLER.getPort());
    public final Joystick buttons = new Joystick(Constants.Controllers.BUTTONS.getPort());

    /* Drive Controls */
    private final int translationAxis = Constants.Controllers.TRANSLATION_AXIS;
    private final int strafeAxis = Constants.Controllers.STRAFE_AXIS;
    private final int rotationAxis = Constants.Controllers.ROTATION_AXIS;
    private final boolean robotRelative = Constants.ROBOT_RELATIVE;

    /* Driver Buttons */
    private final JoystickButton translationButton = new JoystickButton(translationController, Constants.Controllers.TRANSLATION_BUTTON);
    private final JoystickButton rotationButton = new JoystickButton(rotationController, Constants.Controllers.ROTATION_BUTTON);
    private final JoystickButton button1 = new JoystickButton(buttons, 1);
    private final JoystickButton button2 = new JoystickButton(buttons, 2);
    private final JoystickButton button3 = new JoystickButton(buttons,3);
    private final JoystickButton button4 = new JoystickButton(buttons,4);
    private final JoystickButton button5 = new JoystickButton(buttons, 5);
    private final JoystickButton button6 = new JoystickButton(buttons, 6);
    private final JoystickButton button7 = new JoystickButton(buttons, 7);
    private final JoystickButton button8 = new JoystickButton(buttons, 8);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final LED s_LED = new LED();
    private final Vision s_Vision = new Vision(s_Swerve);
    private final Shooter s_Shooter = new Shooter();
    private final Intake s_Intake = new Intake();


    /* Commands */
    public final Music c_Music = new Music(s_Swerve);
    private final Align c_Align = new Align(s_Swerve, s_Vision, () -> translationController.getRawAxis(translationAxis), () -> translationController.getRawAxis(strafeAxis), () -> robotRelative);
    private final RunNote c_RunNote = new RunNote(s_Intake, s_Shooter, xboxController);


    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("note", new AutoNoteAlign(s_Swerve, s_Vision));
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, () -> translationController.getRawAxis(translationAxis), () -> translationController.getRawAxis(strafeAxis), () -> rotationController.getRawAxis(rotationAxis), () -> robotRelative));
        s_Shooter.setDefaultCommand(c_RunNote);
        autoChooser = AutoBuilder.buildAutoChooser("DO NOTHING");
        Shuffleboard.getTab("Auto").add("Auto", autoChooser).withPosition(0, 0);
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
        /* Driver Buttons */
        button1.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        rotationButton.whileTrue(c_Align);
        button4.onTrue(new InstantCommand(() -> s_Swerve.setNoteTracking(true)));
        button5.onTrue(new InstantCommand(() -> s_Swerve.setNoteTracking(false)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Logger.recordOutput("Auto", autoChooser.getSelected().toString());
        return autoChooser.getSelected();
    }
}