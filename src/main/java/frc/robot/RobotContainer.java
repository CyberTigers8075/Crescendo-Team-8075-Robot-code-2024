package frc.robot;

import java.net.InetSocketAddress;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
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
    public final static Joystick driver = new Joystick(0);

    /* Drive Controls */
    public final static int translationAxis = XboxController.Axis.kLeftY.value;
    public final int strafeAxis = XboxController.Axis.kLeftX.value;
    public final static int rotationAxis = XboxController.Axis.kRightX.value;
    // Cameras
    //public Camera camera = new Camera();
    //public Camera cameraDos = new Camera();
    //Joysticks
    public static Joystick mechJoystick = new Joystick(Constants.MECH_JOYSTICK);
    // Buttons
    private Trigger mechA = new JoystickButton(mechJoystick, Constants.A);
    private Trigger mechB = new JoystickButton(mechJoystick, Constants.B);
    private Trigger mechY = new JoystickButton(mechJoystick, Constants.Y);
    public static Trigger mechX = new JoystickButton(mechJoystick, Constants.X);
    private Trigger rightButton = new JoystickButton(mechJoystick, Constants.RB);
    private Trigger leftButton = new JoystickButton(mechJoystick, Constants.LB);
    private Trigger startButton = new JoystickButton(mechJoystick, Constants.START);
    private Trigger backButton = new JoystickButton(mechJoystick, Constants.BACK);
    private Trigger driverA = new JoystickButton(driver, Constants.A);
    private Trigger rightTrigger = new JoystickButton(mechJoystick, Constants.RT);
    private Trigger leftTrigger = new JoystickButton(mechJoystick, Constants.LT);
    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Scorpion scorpion = new Scorpion();
    public static final Intake intake = new Intake();
    public static final Climber climber = new Climber();
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            ));
            climber.setDefaultCommand(new Climb(climber));
            intake.setDefaultCommand(new IntakeControls(intake));
        
        
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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        /*Mech Buttons */
        mechX.onTrue(new IntakeDown(scorpion));
        mechA.onTrue(new IntakeLift(scorpion));
        //mechB.onTrue(new IntakeFlap(scorpion));
        mechY.onTrue(new Safety(climber));
        //rightTrigger.whileTrue(new IntakeControls(intake));
        //leftTrigger.whileTrue(new IntakeControls(intake));
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new Auto(s_Swerve);
        // An ExampleCommand will run in autonomous
    }
        
}