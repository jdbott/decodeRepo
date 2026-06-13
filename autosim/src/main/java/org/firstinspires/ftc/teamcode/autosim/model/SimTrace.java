package org.firstinspires.ftc.teamcode.autosim.model;

import java.util.ArrayList;
import java.util.List;

/** The full trace: the single artifact the sim core hands the viewer. */
public final class SimTrace {
    public Meta meta = new Meta();
    public FieldSpec field = new FieldSpec();
    public RobotSpec robot = new RobotSpec();
    public List<PathSpec> paths = new ArrayList<>();
    public List<Frame> frames = new ArrayList<>();
    public List<ActionEvent> events = new ArrayList<>();
}
