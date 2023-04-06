package frc.lib.util.pathplanner;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.FieldConstants;

/**
 * Custom stuff for pathplanner
 */
public class PathPlannerTrajectory1 extends PathPlannerTrajectory {

    /** 
     * Flip path to break PP's convention
     */
    public static PathPlannerState transformStateForAlliance(PathPlannerState state,
        DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            // Create a new state so that we don't overwrite the original
            PathPlannerState transformedState = state;

            // Translation2d transformedTranslation = new Translation2d(state.poseMeters.getX(),
            // FIELD_WIDTH_METERS - state.poseMeters.getY());
            // Rotation2d transformedHeading = state.poseMeters.getRotation().times(-1);
            // The holonomic heading needs to be negated and rotated
            Rotation2d transformedHolonomicRotation =
                state.holonomicRotation.times(-1).plus(Rotation2d.fromDegrees(180));

            transformedState.timeSeconds = state.timeSeconds;
            transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
            transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
            transformedState.poseMeters = FieldConstants.allianceFlip(state.poseMeters);
            transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
            transformedState.holonomicRotation = transformedHolonomicRotation;
            transformedState.holonomicAngularVelocityRadPerSec =
                -state.holonomicAngularVelocityRadPerSec;
            // transformedState.curveRadius = -state.curveRadius;
            transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;
            // transformedState.deltaPos = state.deltaPos;

            return transformedState;
        } else {
            return state;
        }
    }
    

    /** 
     * Flip path to break PP's convention
     */
    public static PathPlannerTrajectory transformTrajectoryForAlliance(
        PathPlannerTrajectory trajectory, DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            List<State> transformedStates = new ArrayList<>();

            for (State s : trajectory.getStates()) {
                PathPlannerState state = (PathPlannerState) s;

                transformedStates.add(transformStateForAlliance(state, alliance));
            }
            return new PathPlannerTrajectory(transformedStates, trajectory.getMarkers(),
                trajectory.getStartStopEvent(), trajectory.getEndStopEvent(), trajectory.fromGUI);
        } else {
            return trajectory;
        }
    }
}
