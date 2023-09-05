package org.frc5572.robotools.checks;

import org.frc5572.robotools.CompilationData;

/** A code checker */
@FunctionalInterface
public interface Check {

    /** Check if code is fine. Returns true if error is fatal. */
    public boolean check(CompilationData data);

}
