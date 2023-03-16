package frc.lib.util;

import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Class containing scoring functions
 */
public class Scoring {


    /**
     * Enum representing game piece
     */
    public static enum GamePiece {
        CONE, CUBE
    }

    /**
     * Get the type of game piece based off the desired scoring position
     *
     * @return Game Piece
     */
    public static GamePiece getGamePiece() {
        return (Robot.column == 1 || Robot.column == 4 || Robot.column == 7) ? GamePiece.CUBE
            : GamePiece.CONE;
    }

    /**
     * Is the game piece a cube
     *
     * @return True if the game piece is a cube
     */
    public static boolean gamePieceIsCube() {
        return getGamePiece() == GamePiece.CUBE;
    }

    /**
     * Get the distance from the goals based on which level and game piece
     *
     * @return distance in Inches from score barrier
     */
    public static double getScoreDistance() {
        GamePiece gamePiece = getGamePiece();
        Map<Integer, Double> xCoord = Map.of();
        if (gamePiece == GamePiece.CUBE) {
            xCoord = Map.of(0, 36.0, 1, 30.0, 2, 29.0);
        } else if (gamePiece == GamePiece.CONE) {
            xCoord = Map.of(0, 26.0, 1, 33.0, 2, 30.0);
        }
        return xCoord.get(Robot.level);
    }

    /**
     * Get the Pose2d to align to score
     *
     * @param xPosition The distance from the line of April Tags
     * @return Position to align to score
     */
    public static Pose2d getScoreAlignment(double xPosition) {
        Pose2d basePos = new Pose2d();
        int column = Robot.column;
        // int level = Robot.level;
        Rotation2d rotation = Rotation2d.fromDegrees(180);
        if (DriverStation.getAlliance() == Alliance.Blue) {
            basePos = FieldConstants.aprilTags.get(8).toPose2d();
        } else if (DriverStation.getAlliance() == Alliance.Red) {
            basePos = FieldConstants.aprilTags.get(3).toPose2d();
        }
        Map<Integer, Translation2d> columns =
            Map.of(0, new Translation2d(xPosition, FieldConstants.Grids.nodeSeparationY * -1), 1,
                new Translation2d(xPosition, FieldConstants.Grids.nodeSeparationY * 0), 2,
                new Translation2d(xPosition, FieldConstants.Grids.nodeSeparationY * 1), 3,
                new Translation2d(xPosition, FieldConstants.Grids.nodeSeparationY * 2), 4,
                new Translation2d(xPosition, FieldConstants.Grids.nodeSeparationY * 3), 5,
                new Translation2d(xPosition, FieldConstants.Grids.nodeSeparationY * 4), 6,
                new Translation2d(xPosition, FieldConstants.Grids.nodeSeparationY * 5), 7,
                new Translation2d(xPosition, FieldConstants.Grids.nodeSeparationY * 6), 8,
                new Translation2d(xPosition, FieldConstants.Grids.nodeSeparationY * 7));
        return basePos.plus(new Transform2d(columns.get(column), rotation));
    }

    /**
     * Get final scoring position to release game piece
     *
     * @return Position to release game piece
     */
    public static Pose2d getScorPose2d() {
        double xPosition = Units.inchesToMeters(getScoreDistance());
        return getScoreAlignment(xPosition);
    }

    /**
     * Get initial position prior to scoring that slightly further back to not hit the score line.
     *
     * @return Posi
     */
    public static Pose2d getPreScorePosition() {
        double xPosition = Units.inchesToMeters(40);
        return getScoreAlignment(xPosition);
    }

    /**
     * Get Arm Angle, Elevator Position, Wrist Angle based on which game piece and level to score
     *
     * @return ArmPosition object
     */
    public static ArmPosition getScoreParameters() {
        GamePiece gamePiece = getGamePiece();
        Map<Integer, Boolean> armExtensionValues = Map.of();
        Map<Integer, Double> armAngleValues = Map.of();
        Map<Integer, Double> wristAngleValues = Map.of();
        // SmartDashboard.putNumber("Targeted Level", Robot.level);
        // SmartDashboard.putNumber("Targeted Column", Robot.column);
        SmartDashboard.putString("Targeted Game Piece", gamePiece.toString());

        if (gamePiece == GamePiece.CUBE) {
            armExtensionValues = Map.of(0, false, 1, false, 2, false);
            armAngleValues = Map.of(0, -70.0, 1, 0.0, 2, 0.0);
            wristAngleValues = Map.of(0, 60.0, 1, -80.0, 2, -20.0);
        } else if (gamePiece == GamePiece.CONE) {

            armExtensionValues = Map.of(0, false, 1, false, 2, true);
            armAngleValues = Map.of(0, -70.0, 1, 7.0, 2, 16.0);
            wristAngleValues = Map.of(0, 60.0, 1, -60.0, 2, -55.0);
        }
        return new ArmPosition(armAngleValues.get(Robot.level), armExtensionValues.get(Robot.level),
            wristAngleValues.get(Robot.level));
    }
}
