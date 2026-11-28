package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.resources.FieldTarget;
import org.firstinspires.ftc.teamcode.resources.FieldSideTargetSelector;

@TeleOp(name = "BIOBUZZ Red", group = "BIOBUZZ")
public class RedTeleOp extends Test {
    // Selected red CELL points from the team's visualizer. Not a live CELL-state estimate.
    // Pivot offset is the team's approximate measurement: 4 in forward, centered laterally.
    private static final FieldTarget RED_LOW_Y_CELL = new FieldTarget(
            "RED lower-Y CELL (57.53, 58.87)", 57.53, 58.87, 4.0, 0.0);
    private static final FieldTarget RED_HIGH_Y_CELL = new FieldTarget(
            "RED higher-Y CELL (57.53, 85.21)", 57.53, 85.21, 4.0, 0.0);
    // The screenshot's (22.34, 71.03) is a point ON the horizontal split line,
    // not a CELL target. Robot center Y selects an approach within the RED alliance.
    public static final double CELL_SPLIT_Y_IN = 71.03;
    public static final double CELL_SWITCH_HYSTERESIS_IN = 2.0;
    private final FieldSideTargetSelector cellSelector = new FieldSideTargetSelector(
            RED_LOW_Y_CELL, RED_HIGH_Y_CELL, CELL_SPLIT_Y_IN, CELL_SWITCH_HYSTERESIS_IN);

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected Pose getFieldStartPose() {
        return PoseFactory.degrees().of(30.7228246331, 62.7353389114, 0);
    }

    @Override
    protected FieldTarget getPreAimTarget() {
        return cellSelector.getSelectedTarget();
    }

    @Override
    protected void updatePreAimTarget(Pose pose, boolean reset) {
        if (reset) cellSelector.reset(pose.y());
        else cellSelector.select(pose.y());
    }

    @Override
    protected String getPreAimSelectionRule() {
        return "POSE SIDE ONLY: switch LOW at Y <= 69.03; HIGH at Y >= 73.03 in";
    }
}
