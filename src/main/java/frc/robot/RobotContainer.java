// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.arm.ArmMoving;
import frc.robot.commands.dropintake.MoveDDIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DropIntake;
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
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(Constants.DRIVER_ID);
    private final CommandXboxController operator = new CommandXboxController(Constants.OPERATOR_ID);



    // Initialize AutoChooser Sendable
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    public PneumaticHub ph = new PneumaticHub();

    // Field Relative and openLoop Variables
    boolean fieldRelative;
    boolean openLoop;
    int ledPattern = 0;

    // Subsystems
    private LEDs leds = new LEDs(Constants.LEDConstants.LED_COUNT, Constants.LEDConstants.PWM_PORT);
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final DropIntake dIntake = new DropIntake();
    private final Arm s_Arm = new Arm();
    private final WristIntake wrist;
    // public DigitalInput testSensor = new DigitalInput(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        ph.enableCompressorAnalog(90, 120);
        wrist = new WristIntake(ph);
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver,
            Constants.Swerve.IS_FIELD_RELATIVE, Constants.Swerve.IS_OPEN_LOOP));
        // autoChooser.addOption(resnickAuto, new ResnickAuto(s_Swerve));
        SmartDashboard.putData("Choose Auto: ", autoChooser);
        // Configure the button bindings
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
        // driver.y().onTrue(new InstantCommand(() -> s_Swerve.resetFieldRelativeOffset()));
        // driver.x().whileTrue(new TestTransform(s_Swerve,
        // new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(180)), 6));
        // driver.a().onTrue(new InstantCommand(() -> s_Swerve.resetInitialized()));
        // driver.rightTrigger().whileTrue(new RainbowLEDs(leds));
        // driver.leftTrigger().whileTrue(new PoliceLEDs(leds));
        // driver.start().onTrue(new DisabledInstantCommand(() -> this.ledPattern = 0));
        // driver.povDown().onTrue(new DisabledInstantCommand(() -> this.ledPattern = 1));
        // driver.povRight().onTrue(new DisabledInstantCommand(() -> this.ledPattern = 2));
        // driver.povLeft().onTrue(new DisabledInstantCommand(() -> this.ledPattern = 3));

        // /* Operator Buttons */
        // operator.leftTrigger().onTrue(new FlashingLEDColor(leds, Color.kYellow)
        // .until(() -> this.testSensor.get()).withTimeout(5.0));
        // operator.rightTrigger().onTrue(new FlashingLEDColor(leds, Color.kPurple)
        // .until(() -> this.testSensor.get()).withTimeout(5.0));

        /* Triggers */
        // Trigger grabbedGamePiece = new Trigger(() -> this.testSensor.get());
        // new Trigger(() -> this.ledPattern == 1).whileTrue(new RainbowLEDs(leds));
        // new Trigger(() -> this.ledPattern == 2).whileTrue(new PoliceLEDs(leds));
        // new Trigger(() -> this.ledPattern == 3)
        // .whileTrue(new FlashingLEDColor(leds, Color.kGhostWhite, Color.kGreen));
        // grabbedGamePiece.whileTrue(
        // new DisabledInstantCommand(() -> leds.setColor(Color.kGreen), leds).repeatedly());
        // grabbedGamePiece.negate().whileTrue(new FlashingLEDColor(leds,
        // Color.kBlue).withTimeout(3));
        // operator.povDown().whileTrue(new MorseCodeFlash(leds, "ROSBOTS"));

        driver.y().onTrue(new InstantCommand(
            () -> SmartDashboard.putString(" .get ABS: ", dIntake.getAngleMeasurement() + " ")));

        driver.b().whileTrue(new MoveDDIntake(dIntake, dIntake.position1));
        driver.a().whileTrue(new MoveDDIntake(dIntake, dIntake.position2));
        driver.x().whileTrue(new MoveDDIntake(dIntake, dIntake.position3));
        // driver.x().whileTrue(new WristIntakeIn(wrist));

        operator.a().whileTrue(new ArmMoving(s_Arm, 90));
        operator.b().whileTrue(new ArmMoving(s_Arm, 3));
        operator.x().whileTrue(new ArmMoving(s_Arm, 120));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
