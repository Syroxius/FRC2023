package frc.robot.commands.drive;

import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.FieldConstants;
import frc.lib.util.Scoring;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;

/**
 * Test April tag transform
 */
public class MoveToScore extends MoveToPos {

    /**
     * Test April tag transform
     */
    public MoveToScore(Swerve swerve) {
        super(swerve);
    }

    @Override
    public void initialize() {
        int column = Robot.column;
        // int level = Robot.level;
        Pose2d basePos = new Pose2d();
        Rotation2d rotation = Rotation2d.fromDegrees(180);
        double xOffset = Units.inchesToMeters(Scoring.getScorePosition());
        // double xOffset = Units.inchesToMeters(50);
        int redInvert = 1;
        if (DriverStation.getAlliance() == Alliance.Blue) {
            basePos = Robot.aprilTagFieldLayout.getTagPose(8).get().toPose2d();
        } else if (DriverStation.getAlliance() == Alliance.Red) {
            basePos = Robot.aprilTagFieldLayout.getTagPose(3).get().toPose2d();
        }
        Map<Integer, Translation2d> columns = Map.of(0,
            new Translation2d(xOffset, FieldConstants.Grids.nodeSeparationY * -1 * redInvert), 1,
            new Translation2d(xOffset, FieldConstants.Grids.nodeSeparationY * 0 * redInvert), 2,
            new Translation2d(xOffset, FieldConstants.Grids.nodeSeparationY * 1 * redInvert), 3,
            new Translation2d(xOffset, FieldConstants.Grids.nodeSeparationY * 2 * redInvert), 4,
            new Translation2d(xOffset, FieldConstants.Grids.nodeSeparationY * 3 * redInvert), 5,
            new Translation2d(xOffset, FieldConstants.Grids.nodeSeparationY * 4 * redInvert), 6,
            new Translation2d(xOffset, FieldConstants.Grids.nodeSeparationY * 5 * redInvert), 7,
            new Translation2d(xOffset, FieldConstants.Grids.nodeSeparationY * 6 * redInvert), 8,
            new Translation2d(xOffset, FieldConstants.Grids.nodeSeparationY * 7 * redInvert));
        this.pose2d = basePos.plus(new Transform2d(columns.get(column), rotation));
        swerve.resetOdometry(this.pose2d);
    }
}
