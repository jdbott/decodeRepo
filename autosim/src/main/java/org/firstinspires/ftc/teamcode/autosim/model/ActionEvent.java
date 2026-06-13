package org.firstinspires.ftc.teamcode.autosim.model;

import org.firstinspires.ftc.teamcode.autosim.geom.Pose2d;

/** A sparse action event. {@code durationMs} is always sourced from the FSM's own timing. */
public final class ActionEvent {
    public long tMs;
    public Category category;
    public String label;
    public long durationMs;
    public Pose2d pose;            // optional

    public ActionEvent(long tMs, Category category, String label, long durationMs, Pose2d pose) {
        this.tMs = tMs;
        this.category = category;
        this.label = label;
        this.durationMs = durationMs;
        this.pose = pose;
    }
}
