package org.firstinspires.ftc.teamcode.resources;

/** A selected field point for geometric pre-aim; not evidence of CELL state. */
public final class FieldTarget {
    public final String name;
    public final double xIn;
    public final double yIn;
    public final double pivotForwardIn;
    public final double pivotLeftIn;

    public FieldTarget(String name, double xIn, double yIn,
                       double pivotForwardIn, double pivotLeftIn) {
        this.name = name;
        this.xIn = xIn;
        this.yIn = yIn;
        this.pivotForwardIn = pivotForwardIn;
        this.pivotLeftIn = pivotLeftIn;
    }

    /** Degrees relative to chassis forward, positive left/CCW. */
    public double relativeBearingDegrees(double robotXIn, double robotYIn, double headingRad) {
        if (!Double.isFinite(robotXIn) || !Double.isFinite(robotYIn)
                || !Double.isFinite(headingRad) || !Double.isFinite(xIn)
                || !Double.isFinite(yIn) || !Double.isFinite(pivotForwardIn)
                || !Double.isFinite(pivotLeftIn)) return Double.NaN;
        double c = Math.cos(headingRad), s = Math.sin(headingRad);
        double pivotX = robotXIn + pivotForwardIn * c - pivotLeftIn * s;
        double pivotY = robotYIn + pivotForwardIn * s + pivotLeftIn * c;
        double dx = xIn - pivotX, dy = yIn - pivotY;
        if (Math.hypot(dx, dy) < 1.0) return Double.NaN;
        double relative = Math.atan2(dy, dx) - headingRad;
        return Math.toDegrees(Math.atan2(Math.sin(relative), Math.cos(relative)));
    }
}
