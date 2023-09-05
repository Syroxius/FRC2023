package org.frc5572.robotools;

import javax.annotation.processing.Messager;
import javax.annotation.processing.ProcessingEnvironment;
import javax.lang.model.element.Element;
import javax.lang.model.element.TypeElement;
import javax.lang.model.util.Types;
import javax.tools.Diagnostic;
import com.sun.source.tree.CompilationUnitTree;
import com.sun.source.tree.LineMap;
import com.sun.source.tree.Tree;
import com.sun.source.util.JavacTask;
import com.sun.source.util.SourcePositions;
import com.sun.source.util.TaskEvent;
import com.sun.source.util.Trees;

/**
 * Wrapper around a TypeElement. Includes helpers for printing errors, warnings etc.
 */
public class CompilationData {

    /** Type Information for entire classpath. */
    public Types types;
    private Trees trees;
    private SourcePositions positions;
    private CompilationUnitTree compilationUnitTree;
    /** Element being processed */
    public TypeElement element;
    private Messager messager;

    /** Basic constructor. */
    public CompilationData(Types types, Trees trees, SourcePositions positions,
        CompilationUnitTree compilationUnitTree, TypeElement element, Messager messager) {
        this.types = types;
        this.trees = trees;
        this.positions = positions;
        this.compilationUnitTree = compilationUnitTree;
        this.element = element;
        this.messager = messager;
    }

    /** Constructor from Javac Plugin context. */
    public CompilationData(JavacTask task, TaskEvent event) {
        this(task.getTypes(), Trees.instance(task), Trees.instance(task).getSourcePositions(),
            event.getCompilationUnit(), event.getTypeElement(), null);
    }

    /** Constructor from Annotation Processor context. */
    public CompilationData(ProcessingEnvironment processingEnv, TypeElement element) {
        this(processingEnv.getTypeUtils(), null, null, null, element, processingEnv.getMessager());
    }

    /** Show error */
    public void error(Element element, Object object) {
        echo(element, object.toString(), Diagnostic.Kind.ERROR, "error", "Error");
    }

    /** Show warning */
    public void warn(Element element, Object object) {
        echo(element, object.toString(), Diagnostic.Kind.MANDATORY_WARNING, "warning", "Warning");
    }

    /** Show note (this doesn't work in VS Code yet) */
    public void note(Element element, Object object) {
        echo(element, object.toString(), Diagnostic.Kind.NOTE, "notice", "Note");
    }

    private void echo(Element element, String errString, Diagnostic.Kind kind, String ghString,
        String humanString) {
        if (compilationUnitTree == null) {
            messager.printMessage(kind, errString, element);
        } else {
            Tree tree = trees.getTree(element);
            LineMap linemap = compilationUnitTree.getLineMap();
            long pos = positions.getStartPosition(compilationUnitTree, tree);
            long row = linemap.getLineNumber(pos);
            String name = compilationUnitTree.getSourceFile().toUri().toString().split("/src/")[1];
            System.out
                .println("::" + ghString + " file=src/" + name + ",line=" + row + "::" + errString);
        }
    }

    private boolean _implements(String qualifiedName, TypeElement elem) {
        if (elem.getQualifiedName().toString().equals(qualifiedName)) {
            return true;
        }
        Element superClass = types.asElement(elem.getSuperclass());
        if (superClass instanceof TypeElement) {
            if (_implements(qualifiedName, (TypeElement) superClass)) {
                return true;
            }
        }
        for (var iface : elem.getInterfaces()) {
            Element interface_ = types.asElement(iface);
            if (interface_ instanceof TypeElement) {
                if (_implements(qualifiedName, (TypeElement) interface_)) {
                    return true;
                }
            }
        }
        return false;
    }

    private boolean _extends(String qualifiedName, TypeElement elem) {
        if (elem.getQualifiedName().toString().equals(qualifiedName)) {
            return true;
        }
        Element superClass = types.asElement(elem.getSuperclass());
        if (superClass instanceof TypeElement) {
            if (_extends(qualifiedName, (TypeElement) superClass)) {
                return true;
            }
        }
        return false;
    }

    /** Get if this type implements an interface by name. */
    public boolean implementsInterface(String qualifiedName) {
        return _implements(qualifiedName, this.element);
    }

    /** Get if the specified type implements an interface by name. */
    public boolean implementsInterface(TypeElement e, String qualifiedName) {
        return _implements(qualifiedName, e);
    }

    /** Get if this type extends a class by name. */
    public boolean extendsClass(String qualifiedName) {
        return _extends(qualifiedName, this.element);
    }

    /** Get if the specified type extends a class by name. */
    public boolean extendsClass(TypeElement e, String qualifiedName) {
        return _extends(qualifiedName, e);
    }

}
