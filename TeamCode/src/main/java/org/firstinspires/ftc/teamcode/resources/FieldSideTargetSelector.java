package org.firstinspires.ftc.teamcode.resources;

/** Selects a pose-only approach target. Does not infer alliance or scorable CELL state. */
public final class FieldSideTargetSelector {
    private final FieldTarget lowYTarget;
    private final FieldTarget highYTarget;
    private final double splitYIn;
    private final double hysteresisIn;
    private FieldTarget selected;

    public FieldSideTargetSelector(FieldTarget lowYTarget, FieldTarget highYTarget,
                                   double splitYIn, double hysteresisIn) {
        if (lowYTarget == null || highYTarget == null || lowYTarget == highYTarget
                || !Double.isFinite(splitYIn) || !Double.isFinite(hysteresisIn)
                || hysteresisIn < 0) throw new IllegalArgumentException("Invalid side selector configuration");
        this.lowYTarget = lowYTarget;
        this.highYTarget = highYTarget;
        this.splitYIn = splitYIn;
        this.hysteresisIn = hysteresisIn;
    }

    public FieldTarget reset(double robotYIn) {
        selected = null;
        return select(robotYIn);
    }

    public FieldTarget select(double robotYIn) {
        if (!Double.isFinite(robotYIn)) return null;
        if (selected == null) selected = robotYIn >= splitYIn ? highYTarget : lowYTarget;
        else if (selected == lowYTarget && robotYIn >= splitYIn + hysteresisIn) selected = highYTarget;
        else if (selected == highYTarget && robotYIn <= splitYIn - hysteresisIn) selected = lowYTarget;
        return selected;
    }

    public FieldTarget getSelectedTarget() { return selected; }
}
