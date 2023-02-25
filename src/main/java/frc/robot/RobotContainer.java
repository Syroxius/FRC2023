package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DisabledInstantCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.leds.FlashingLEDColor;
import frc.robot.commands.leds.RainbowLEDs;
import frc.robot.commands.wrist.WristIntakeIn;
import frc.robot.commands.wrist.WristIntakePanic;
import frc.robot.commands.wrist.WristIntakeRelease;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DropIntake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.WristIntake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */


    private final CommandXboxController driver = new CommandXboxController(Constants.DRIVER_ID);
    private final CommandXboxController operator = new CommandXboxController(Constants.OPERATOR_ID);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    public PneumaticHub ph = new PneumaticHub();

    // Field Relative and openLoop Variables
    boolean fieldRelative;
    boolean openLoop;
    int ledPattern = 0;

    // Subsystems
    private LEDs leds = new LEDs(Constants.LEDConstants.LED_COUNT, Constants.LEDConstants.PWM_PORT);
    /* Subsystems */
    private final Wrist wrist = new Wrist();
    private final WristIntake wristIntake = new WristIntake(ph);
    private final Swerve s_Swerve = new Swerve();
    private final DropIntake dIntake = new DropIntake();
    private final Arm s_Arm = new Arm();
    // public DigitalInput testSensor = new DigitalInput(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        ph.enableCompressorAnalog(90, 120);
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver,
            Constants.Swerve.IS_FIELD_RELATIVE, Constants.Swerve.IS_OPEN_LOOP));
        // autoChooser.addOption(resnickAuto, new ResnickAuto(s_Swerve));
        SmartDashboard.putData("Choose Auto: ", autoChooser);
        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1));
        // Configure the button bindings
        leds.setDefaultCommand(new RainbowLEDs(leds));
        configureButtonBindings();

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.resetFieldRelativeOffset()));
        // driver.x().whileTrue(new TestTransform(s_Swerve,
        // new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(180)), 6));
        // driver.a().onTrue(new InstantCommand(() -> s_Swerve.resetInitialized()));
        // driver.rightTrigger().whileTrue(new RainbowLEDs(leds));
        // driver.leftTrigger().whileTrue(new PoliceLEDs(leds));
        // driver.start().onTrue(new DisabledInstantCommand(() -> this.ledPattern = 0));
        // driver.povDown().onTrue(new DisabledInstantCommand(() -> this.ledPattern = 1));
        // driver.povRight().onTrue(new DisabledInstantCommand(() -> this.ledPattern = 2));
        // driver.povLeft().onTrue(new DisabledInstantCommand(() -> this.ledPattern = 3));

        /* Operator Buttons */
        operator.leftBumper().onTrue(new FlashingLEDColor(leds, Color.kYellow)
            .until(() -> this.wristIntake.getConeSensor()).withTimeout(5.0));
        operator.rightBumper().onTrue(new FlashingLEDColor(leds, Color.kPurple)
            .until(() -> this.wristIntake.getCubeSensor()).withTimeout(5.0));
        operator.a().whileTrue(new WristIntakeIn(wristIntake));
        operator.b().whileTrue(new WristIntakeRelease(wristIntake));


        /* Triggers */
        // new Trigger(() -> this.ledPattern == 1).whileTrue(new RainbowLEDs(leds));
        // new Trigger(() -> this.ledPattern == 2).whileTrue(new PoliceLEDs(leds));
        // new Trigger(() -> this.ledPattern == 3)
        // .whileTrue(new FlashingLEDColor(leds, Color.kGhostWhite, Color.kGreen));

        Trigger grabbedGamePiece =
            new Trigger(() -> this.wristIntake.getConeSensor() || this.wristIntake.getCubeSensor());
        grabbedGamePiece.whileTrue(
            new DisabledInstantCommand(() -> leds.setColor(Color.kGreen), leds).repeatedly());
        grabbedGamePiece.onFalse(new FlashingLEDColor(leds, Color.kBlue).withTimeout(3));
        Trigger intakePanic =
            new Trigger(() -> this.wristIntake.getConeSensor() && this.wristIntake.getCubeSensor());
        intakePanic.whileTrue(new WristIntakePanic(wristIntake));
        // operator.povDown().whileTrue(new MorseCodeFlash(leds, "ROSBOTS"));

        // driver.y().onTrue(new InstantCommand(
        // () -> SmartDashboard.putString(" .get ABS: ", dIntake.getAngleMeasurement() + " ")));

        // driver.b().whileTrue(new MoveDDIntake(dIntake, dIntake.position1));
        // driver.a().whileTrue(new MoveDDIntake(dIntake, dIntake.position2));
        // driver.x().whileTrue(new MoveDDIntake(dIntake, dIntake.position3));
        // // driver.x().whileTrue(new WristIntakeIn(wrist));

        // operator.a().whileTrue(new ArmMoving(s_Arm, 90));
        // operator.b().whileTrue(new ArmMoving(s_Arm, 3));
        // operator.x().whileTrue(new ArmMoving(s_Arm, 120));

        // operator.leftTrigger().whileTrue(new FunctionalCommand(() -> wrist.lastAngle = 0,
        // () -> wrist.test(), inter -> wrist.wristMotor.set(0), () -> false, wrist));
    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        // return new P1_3B(swerveDrive, shooter, innerMagazine, outerMagazine, intake, turret,
        // vision);
        return autoChooser.getSelected();
    }
}
