package org.frc5572.robotools;

import org.frc5572.robotools.checks.Check;
import org.frc5572.robotools.checks.IOCheck;

/** All checks to run. */
public class Checks {

    private static final Check[] CHECKS = new Check[] {new IOCheck()};

    /** Run through all checks */
    public static boolean process(CompilationData data) {
        boolean fatal = false;
        for (Check c : CHECKS) {
            fatal = fatal || c.check(data);
        }
        return fatal;
    }

}
