package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.FieldConstants;
import frc.robot.commands.drive.MoveToPos;
import frc.robot.subsystems.Swerve;

/**
 * Leaves community area.
 */
public class LeaveCommunity extends MoveToPos {

    public static double distanceFromTape = 240.0;

    /**
     * Leaves community
     *
     * @param swerve swerve
     */
    public LeaveCommunity(Swerve swerve) {
        super(swerve,
            () -> new Pose2d(
                new Translation2d(Units.inchesToMeters(distanceFromTape), swerve.getPose().getY()),
                Rotation2d.fromDegrees(0)),
            true);
    }

    @Override
    public boolean isFinished() {
        Pose2d pose2d = FieldConstants.allianceFlip(swerve.getPose());
        return pose2d.getX() > Units.inchesToMeters(distanceFromTape - 10);
    }
}
