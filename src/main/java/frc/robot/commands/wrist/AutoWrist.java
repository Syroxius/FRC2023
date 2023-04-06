package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.WristIntake;

/**
 * Consistent wrist operation for Autonomous.
 */
public class AutoWrist extends CommandBase {

    private WristIntake intake;
    private double speed;


    private AutoWrist(WristIntake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        this.addRequirements(intake);
    }

    /**
     * Intake for cube
     */
    public static AutoWrist cubeIntake(WristIntake intake, double speed) {
        return new AutoWrist(intake, speed);
    }

    /**
     * Intake for cube
     */
    public static AutoWrist cubeIntake(WristIntake intake) {
        return cubeIntake(intake, Constants.Wrist.INTAKE_SPEED);
    }

    /**
     * Outtake for cube
     */
    public static AutoWrist cubeOuttake(WristIntake intake, double speed) {
        return cubeIntake(intake, -speed);
    }

    /**
     * Outtake for cube
     */
    public static AutoWrist cubeOuttake(WristIntake intake) {
        return cubeOuttake(intake, Constants.Wrist.INTAKE_RELEASE_SPEED);
    }

    /**
     * Intake for cone
     */
    public static AutoWrist coneIntake(WristIntake intake, double speed) {
        return cubeIntake(intake, -speed);
    }

    /**
     * Intake for cone
     */
    public static AutoWrist coneIntake(WristIntake intake) {
        return coneIntake(intake, Constants.Wrist.INTAKE_SPEED);
    }

    /**
     * Outtake for cone
     */
    public static AutoWrist coneOuttake(WristIntake intake, double speed) {
        return cubeIntake(intake, speed);
    }

    /**
     * Outtake for cone
     */
    public static AutoWrist coneOuttake(WristIntake intake) {
        return coneOuttake(intake, Constants.Wrist.INTAKE_RELEASE_SPEED);
    }

    @Override
    public void initialize() {
        intake.setMotorRaw(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotorRaw(Constants.Wrist.INTAKE_STOP_SPEED);
    }

}
