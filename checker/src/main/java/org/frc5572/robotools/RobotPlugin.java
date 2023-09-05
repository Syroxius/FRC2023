package org.frc5572.robotools;

import javax.lang.model.util.Types;
import com.sun.source.util.JavacTask;
import com.sun.source.util.Plugin;
import com.sun.source.util.SourcePositions;
import com.sun.source.util.TaskEvent;
import com.sun.source.util.TaskListener;
import com.sun.source.util.Trees;

/**
 * Javac plugin has source info, so it's used for Github Action annotations.
 */
public class RobotPlugin implements Plugin {

    /**
     * Name used in build.gradle
     */
    @Override
    public String getName() {
        return "rchk";
    }

    /**
     * Function run when loaded
     */
    @Override
    public void init(JavacTask task, String... arg1) {
        Types types = task.getTypes();
        Trees trees = Trees.instance(task);
        SourcePositions positions = trees.getSourcePositions();
        task.addTaskListener(new TaskListener() {
            @Override
            public void finished(TaskEvent event) {
                if (event.getKind() == TaskEvent.Kind.ANALYZE) {
                    CompilationData data = new CompilationData(types, trees, positions,
                        event.getCompilationUnit(), event.getTypeElement(), null);
                    Checks.process(data);
                }
            }
        });
    }

}
