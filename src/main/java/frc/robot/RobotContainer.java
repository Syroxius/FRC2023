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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.DisabledInstantCommand;
import frc.lib.util.Scoring;
import frc.lib.util.Scoring.GamePiece;
import frc.robot.autos.CrossAndDock;
import frc.robot.autos.LeaveCommunity;
import frc.robot.autos.Score1;
import frc.robot.autos.Score1Dock;
import frc.robot.commands.arm.ConeIntake;
import frc.robot.commands.arm.CubeIntake;
import frc.robot.commands.arm.DockArm;
import frc.robot.commands.arm.ScoreArm;
import frc.robot.commands.drive.MoveToEngage;
import frc.robot.commands.drive.MoveToScore;
import frc.robot.commands.drive.TeleopSwerve;
import frc.robot.commands.leds.FlashingLEDColor;
import frc.robot.commands.leds.MovingColorLEDs;
import frc.robot.commands.leds.PoliceLEDs;
import frc.robot.commands.wrist.VariableIntake;
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
    public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    private ShuffleboardLayout targetGrid =
        RobotContainer.mainDriverTab.getLayout("Next Position", BuiltInLayouts.kGrid)
            .withPosition(6, 0).withSize(2, 4).withProperties(
                Map.of("Number of columns", 2, "Number of rows", 1, "Label position", "TOP"));
    public GenericEntry levelWidget = targetGrid.add("Level", Robot.level)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Min", -1, "Max", 2, "Center",
            -1, "Num tick marks", 4, "Show Text", false, "Orientation", "VERTICAL"))
        .getEntry();
    public GenericEntry columnWidget = targetGrid.add("Column", Robot.column)
        .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Min", 0, "Max", 8, "Center",
            0, "Num tick marks", 5, "Show Text", false, "Orientation", "VERTICAL"))
        .getEntry();
    public GenericEntry gamePieceWidget =
        mainDriverTab.add("Game Piece", Scoring.getGamePiece() == GamePiece.CONE)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "yellow", "Color when false", "purple"))
            .withPosition(6, 4).withSize(2, 1).getEntry();

    private final SendableChooser<Integer> levelsChooser = new SendableChooser<>();
    private final SendableChooser<Integer> columnsChooser = new SendableChooser<>();

    public ComplexWidget autoLevelWidget =
        autoTab.add("Level", levelsChooser).withWidget(BuiltInWidgets.kComboBoxChooser)
            .withProperties(Map.of("Show Glyph", true, "Glyph", "ARROWS_V")).withPosition(8, 2)
            .withSize(2, 2);
    public ComplexWidget autoColumnWidet =
        autoTab.add("Column", columnsChooser).withWidget(BuiltInWidgets.kComboBoxChooser)
            .withProperties(Map.of("Show Glyph", true, "Glyph", "ARROWS_H")).withPosition(8, 0)
            .withSize(2, 2);
    public static GenericEntry enableDockWidget = autoTab.add("Enable Dock", true)
        .withWidget(BuiltInWidgets.kToggleSwitch).withPosition(10, 1).withSize(2, 1).getEntry();
    public ComplexWidget cameraFeed = mainDriverTab.add("Camera Feed", Robot.camera)
        .withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 0).withSize(6, 5).withProperties(
            Map.of("Show crosshair", false, "Show controls", false, "Rotation", "QUARTER_CCW"));

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    public ComplexWidget autoChooserWidget = autoTab.add("Auto Chooser", autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(10, 0).withSize(2, 1);

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
    public final Swerve s_Swerve = new Swerve();
    // private final DropIntake s_dIntake = new DropIntake();
    private final Arm s_Arm = new Arm(ph);
    private final WristIntake s_wristIntake = new WristIntake();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        ph.enableCompressorAnalog(90, 120);
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver,
            Constants.Swerve.IS_FIELD_RELATIVE, Constants.Swerve.IS_OPEN_LOOP, s_Arm));
        s_wristIntake.setDefaultCommand(new VariableIntake(s_wristIntake, operator));
        // autoChooser.addOption(resnickAuto, new ResnickAuto(s_Swerve));
        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1));
        autoChooser.addOption("Leave Community", new LeaveCommunity(s_Swerve));
        autoChooser.addOption("Move To Score", new MoveToScore(s_Swerve, s_Arm, s_wristIntake));

        levelsChooser.setDefaultOption("0", 0);
        levelsChooser.addOption("-1", -1);
        levelsChooser.addOption("0", 0);
        levelsChooser.addOption("1", 1);
        levelsChooser.addOption("2", 2);

        columnsChooser.setDefaultOption("0", 0);
        columnsChooser.addOption("0", 0);
        columnsChooser.addOption("1", 1);
        columnsChooser.addOption("2", 2);
        columnsChooser.addOption("3", 3);
        columnsChooser.addOption("4", 4);
        columnsChooser.addOption("5", 5);
        columnsChooser.addOption("6", 6);
        columnsChooser.addOption("7", 7);
        columnsChooser.addOption("8", 8);
        autoChooser.addOption("Leave Community", new LeaveCommunity(s_Swerve));
        autoChooser.addOption("Cross and Dock", new CrossAndDock(s_Swerve, s_Arm, s_wristIntake));
        autoChooser.addOption("Score 1 Dock", new Score1Dock(s_Swerve, s_Arm, s_wristIntake));
        autoChooser.addOption("Score 1", new Score1(s_Swerve, s_Arm, s_wristIntake));
        // Configure the button bindings
        leds.setDefaultCommand(new MovingColorLEDs(leds, Color.kRed, 8, false));
        // leds.setDefaultCommand(new Twinkle(leds, 60, new Color[] {Color.kRed}));
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

        driver.start().onTrue(new InstantCommand(() -> s_Swerve.resetInitialized()));

        driver.y().onTrue(new DisabledInstantCommand(() -> s_Swerve.resetFieldRelativeOffset()));
        driver.rightTrigger().and(driver.leftTrigger()).and(operator.start())
            .whileTrue(new MoveToScore(s_Swerve, s_Arm, s_wristIntake));
        driver.rightTrigger().and(driver.leftTrigger())
            .onTrue(new InstantCommand(() -> s_Swerve.resetInitialized()));
        driver.rightBumper().and(driver.leftBumper())
            .whileTrue(new MoveToEngage(s_Swerve, s_Arm, s_wristIntake));
        driver.start()
            .whileTrue(new InstantCommand(() -> s_Swerve.wheelsIn(), s_Swerve).repeatedly());

        /* Operator Buttons */
        operator.leftBumper().onTrue(new FlashingLEDColor(leds, Color.kYellow).withTimeout(15.0));
        operator.rightBumper().onTrue(new FlashingLEDColor(leds, Color.kPurple).withTimeout(15.0));

        operator.a().onTrue(new ConeIntake(s_Arm)
            .alongWith(new InstantCommand(() -> s_wristIntake.setInvert(true))));
        operator.b().onTrue(new CubeIntake(s_Arm)
            .alongWith(new InstantCommand(() -> s_wristIntake.setInvert(false))));
        operator.x().onTrue(new ConeIntake(s_Arm));
        operator.y().onTrue(new DockArm(s_Arm, s_wristIntake).withTimeout(.1).repeatedly());

        operator.povUp().onTrue(
            new DisabledInstantCommand(() -> Robot.level = MathUtil.clamp(Robot.level + 1, 0, 2)));
        operator.povDown().onTrue(new DisabledInstantCommand(() -> {
            if (Robot.level <= 0) {
                // Robot.column = 0;
                Robot.level = -1;
            } else {
                Robot.level = MathUtil.clamp(Robot.level - 1, 0, 2);
            }
        }));
        operator.povRight().onTrue(new DisabledInstantCommand(() -> {
            Robot.column = MathUtil.clamp(Robot.column + 1, 0, 8);
            s_wristIntake.setInvert(Scoring.getGamePiece() == Scoring.GamePiece.CONE);
        }));
        operator.povLeft().onTrue(new DisabledInstantCommand(() -> {
            Robot.column = MathUtil.clamp(Robot.column - 1, 0, 8);
            s_wristIntake.setInvert(Scoring.getGamePiece() == Scoring.GamePiece.CONE);
        }));
        operator.rightTrigger().and(operator.leftTrigger())
            .whileTrue(new ScoreArm(s_Arm, s_wristIntake));
        operator.back().toggleOnTrue(new PoliceLEDs(leds));

        // operator.povUp().whileTrue(new MoveArm(s_Arm, 110, 0));
        // operator.povDown().whileTrue(new MoveArm(s_Arm, 45, 0));

        // operator.x().whileTrue(new TestArm(s_Arm));
        // operator.x()
        // .whileTrue(new InstantCommand(() -> System.out.println(s_Arm.getElevatorPosition())));
        // operator.x().whileTrue(new MoveElevator(s_Arm));


        /* TRIGGERs */

        // intakePanic.whileTrue(new WristIntakePanic(s_wristIntake)
        // .deadlineWith(new FlashingLEDColor(leds, Color.kRed)));

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
        Robot.level = levelsChooser.getSelected();
        Robot.column = columnsChooser.getSelected();
        return new DockArm(s_Arm, s_wristIntake).withTimeout(.2).andThen(autoChooser.getSelected());
    }
}
