package frc.lib.util;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
 */
public class DynamicWaitCommand extends CommandBase {
    protected Timer m_timer = new Timer();
    private double m_duration = 0;
    private final DoubleSupplier secondsSupplier;

    /**
     * Creates a new WaitCommand. This command will do nothing, and end after the specified
     * duration.
     *
     * @param seconds the time to wait, in seconds
     */
    public DynamicWaitCommand(DoubleSupplier seconds) {
        this.secondsSupplier = seconds;
    }

    @Override
    public void initialize() {
        m_duration = secondsSupplier.getAsDouble();
        m_timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
