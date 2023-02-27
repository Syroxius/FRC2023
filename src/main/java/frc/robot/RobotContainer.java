package frc.robot;

import java.util.Map;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.DisabledInstantCommand;
import frc.lib.util.Scoring;
import frc.lib.util.Scoring.GamePiece;
import frc.robot.autos.P0;
import frc.robot.commands.arm.ArmIntake;
import frc.robot.commands.arm.DockArm;
import frc.robot.commands.arm.ScoreArm;
import frc.robot.commands.drive.ClimbPlatform;
import frc.robot.commands.drive.MoveToScore;
import frc.robot.commands.drive.TeleopSwerve;
import frc.robot.commands.leds.FlashingLEDColor;
import frc.robot.commands.leds.RainbowLEDs;
import frc.robot.commands.wrist.WristIntakeIn;
import frc.robot.commands.wrist.WristIntakePanic;
import frc.robot.commands.wrist.WristIntakeRelease;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristIntake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Shuffleboard */
    public static ShuffleboardTab mainDriverTab = Shuffleboard.getTab("Main Driver");
    private ShuffleboardLayout targetGrid =
        RobotContainer.mainDriverTab.getLayout("Next Position", BuiltInLayouts.kGrid)
            .withPosition(8, 2).withSize(2, 2).withProperties(
                Map.of("Number of columns", 2, "Number of rows", 1, "Label position", "TOP"));
    public GenericEntry levelWidget = targetGrid.add("Level", Robot.level)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Min", 0, "Max", 2, "Center",
            0, "Num tick marks", 3, "Show Text", false, "Orientation", "VERTICAL"))
        .getEntry();
    public GenericEntry columnWidet = targetGrid.add("Column", Robot.column)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Min", 0, "Max", 8, "Center",
            0, "Num tick marks", 5, "Show Text", false, "Orientation", "VERTICAL"))
        .getEntry();
    public GenericEntry gamePieceWidget =
        mainDriverTab.add("Game Piece", Scoring.getGamePiece() == GamePiece.CONE)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "yellow", "Color when false", "purple"))
            .withPosition(8, 4).withSize(2, 1).getEntry();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    public ComplexWidget autoChooserWidget = mainDriverTab.add("Auto Chooser", autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(2, 4).withSize(2, 1);

    /* Controllers */


    // Initialize AutoChooser Sendable


    private final CommandXboxController driver = new CommandXboxController(Constants.DRIVER_ID);
    private final CommandXboxController operator = new CommandXboxController(Constants.OPERATOR_ID);

    private final PneumaticHub ph = new PneumaticHub();

    // Field Relative and openLoop Variables
    boolean fieldRelative;
    boolean openLoop;
    int ledPattern = 0;

    /* Subsystems */
    private LEDs leds = new LEDs(Constants.LEDConstants.LED_COUNT, Constants.LEDConstants.PWM_PORT);
    private final Swerve s_Swerve = new Swerve();
    // private final DropIntake s_dIntake = new DropIntake();
    private final Arm s_Arm = new Arm();
    private final WristIntake s_wristIntake = new WristIntake(ph);

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
        autoChooser.addOption("Test", new P0(s_Swerve));
        autoChooser.addOption("Move To Score", new MoveToScore(s_Swerve));
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
        driver.leftBumper().and(driver.rightBumper()).whileTrue(new MoveToScore(s_Swerve));
        driver.rightTrigger().and(driver.leftTrigger()).whileTrue(new ClimbPlatform(s_Swerve));

        /* Operator Buttons */
        operator.leftBumper().onTrue(new FlashingLEDColor(leds, Color.kYellow)
            .until(() -> this.s_wristIntake.getConeSensor()).withTimeout(5.0));
        operator.rightBumper().onTrue(new FlashingLEDColor(leds, Color.kPurple)
            .until(() -> this.s_wristIntake.getCubeSensor()).withTimeout(5.0));

        operator.a().whileTrue(new WristIntakeIn(s_wristIntake));
        operator.b().whileTrue(new WristIntakeRelease(s_wristIntake));
        operator.x().whileTrue(new WristIntakeIn(s_wristIntake).alongWith(new ArmIntake(s_Arm)));
        operator.y().whileTrue(new DockArm(s_Arm, s_wristIntake));

        operator.povUp().onTrue(
            new DisabledInstantCommand(() -> Robot.level = MathUtil.clamp(Robot.level + 1, 0, 2)));
        operator.povDown().onTrue(
            new DisabledInstantCommand(() -> Robot.level = MathUtil.clamp(Robot.level - 1, 0, 2)));
        operator.povRight().onTrue(new DisabledInstantCommand(
            () -> Robot.column = MathUtil.clamp(Robot.column + 1, 0, 8)));
        operator.povLeft().onTrue(new DisabledInstantCommand(
            () -> Robot.column = MathUtil.clamp(Robot.column - 1, 0, 8)));
        operator.rightTrigger().and(operator.leftTrigger())
            .whileTrue(new ScoreArm(s_Arm, s_wristIntake));

        // operator.povUp().whileTrue(new MoveArm(s_Arm, 110, 0));
        // operator.povDown().whileTrue(new MoveArm(s_Arm, 45, 0));

        // operator.x().whileTrue(new TestArm(s_Arm));
        // operator.x()
        // .whileTrue(new InstantCommand(() -> System.out.println(s_Arm.getElevatorPosition())));
        // operator.x().whileTrue(new MoveElevator(s_Arm));


        /* TRIGGERs */
        Trigger grabbedGamePiece = new Trigger(
            () -> this.s_wristIntake.getConeSensor() || this.s_wristIntake.getCubeSensor());
        grabbedGamePiece.whileTrue(
            new DisabledInstantCommand(() -> leds.setColor(Color.kGreen), leds).repeatedly());
        grabbedGamePiece.onFalse(new FlashingLEDColor(leds, Color.kBlue).withTimeout(3));
        Trigger intakePanic = new Trigger(
            () -> this.s_wristIntake.getConeSensor() && this.s_wristIntake.getCubeSensor());
        intakePanic.whileTrue(new WristIntakePanic(s_wristIntake)
            .deadlineWith(new FlashingLEDColor(leds, Color.kRed)));

        // driver.y().onTrue(new InstantCommand(
        // () -> SmartDashboard.putString(" .get ABS: ", dIntake.getAngleMeasurement() + " ")));
        // driver.b().whileTrue(new MoveDDIntake(dIntake, dIntake.position1));
        // driver.a().whileTrue(new MoveDDIntake(dIntake, dIntake.position2));
        // driver.x().whileTrue(new MoveDDIntake(dIntake, dIntake.position3));
        // driver.x().whileTrue(new WristIntakeIn(wrist));
        // operator.a().whileTrue(new ArmMoving(s_Arm, 85));
        // operator.b().whileTrue(new ArmMoving(s_Arm, 3));
        // operator.x().whileTrue(new ArmMoving(s_Arm, 120));
        // operator.leftTrigger().whileTrue(new FunctionalCommand(() -> s_wrist.lastAngle = 0,
        // () -> s_wrist.test(), inter -> s_wrist.wristMotor.set(0), () -> false, s_wrist));
        // operator.a().whileTrue(new DockArm(s_Arm, s_dIntake, s_wristIntake));
        // operator.b()
        // .whileTrue(new ScoreArm(s_Arm, s_dIntake, s_wristIntake, 90, s_Arm.elevatorMaxEncoder));
        // operator.leftTrigger().whileTrue(new FunctionalCommand(() -> wrist.lastAngle = 0,
        // () -> wrist.test(), inter -> wrist.wristMotor.set(0), () -> false, wrist));
        // driver.x().whileTrue(new TestTransform(s_Swerve,
        // new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(180)), 6));
        // driver.a().onTrue(new InstantCommand(() -> s_Swerve.resetInitialized()));
        // driver.rightTrigger().whileTrue(new RainbowLEDs(leds));
        // driver.leftTrigger().whileTrue(new PoliceLEDs(leds));
        // driver.start().onTrue(new DisabledInstantCommand(() -> this.ledPattern = 0));
        // driver.povDown().onTrue(new DisabledInstantCommand(() -> this.ledPattern = 1));
        // driver.povRight().onTrue(new DisabledInstantCommand(() -> this.ledPattern = 2));
        // driver.povLeft().onTrue(new DisabledInstantCommand(() -> this.ledPattern = 3));
        // operator.povDown().whileTrue(new MorseCodeFlash(leds, "ROSBOTS"));
        // operator.leftTrigger().whileTrue(
        // new StartEndCommand(() -> s_dIntake.intake(), () -> s_dIntake.stop(), s_dIntake));
        /* Triggers */
        // new Trigger(() -> this.ledPattern == 1).whileTrue(new RainbowLEDs(leds));
        // new Trigger(() -> this.ledPattern == 2).whileTrue(new PoliceLEDs(leds));
        // new Trigger(() -> this.ledPattern == 3)
        // .whileTrue(new FlashingLEDColor(leds, Color.kGhostWhite, Color.kGreen));
        // driver.y().onTrue(new InstantCommand(
        // () -> SmartDashboard.putString(" .get ABS: ", s_dIntake.getAngleMeasurement() + " ")));
        // driver.b().whileTrue(new MoveDDIntake(s_dIntake, s_dIntake.position1));
        // driver.a().whileTrue(new MoveDDIntake(s_dIntake, s_dIntake.position2));
        // driver.x().whileTrue(new MoveDDIntake(s_dIntake, s_dIntake.position3));
        // operator.a().whileTrue(new ArmMoving(s_Arm, 90));
        // operator.b().whileTrue(new ArmMoving(s_Arm, 3));
        // operator.x().whileTrue(new ArmMoving(s_Arm, 120));
        // operator.y().whileTrue(new ElevatorControl(s_Elevator, 2.70));
        // operator.y().whileTrue(new DisabledInstantCommand(
        // () -> System.out.println("ENCODER: " + s_Elevator.getDegreeMeasurement())));
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
