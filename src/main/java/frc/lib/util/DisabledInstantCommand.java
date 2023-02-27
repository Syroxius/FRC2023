package frc.lib.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A Command that runs instantly; it will initialize, execute once, and end on the same iteration of
 * the scheduler. Users can either pass in a Runnable and a set of requirements, or else subclass
 * this command if desired.
 *
 * <p>
 * This command works while the robot is disabled.
 */
public class DisabledInstantCommand extends InstantCommand {

    /**
     * Creates a new InstantCommand that runs the given Runnable with the given requirements.
     *
     * <p>
     * This command works while the robot is disabled.
     *
     * @param toRun the Runnable to run
     * @param requirements the subsystems required by this command
     */
    public DisabledInstantCommand(Runnable toRun, Subsystem... requirements) {
        super(toRun, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
