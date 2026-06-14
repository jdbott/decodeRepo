package org.firstinspires.ftc.teamcode.autosim.model;

import org.firstinspires.ftc.teamcode.autosim.geom.Pose2d;

/** A dense per-tick sample for smooth playback (one start frame in Phase 1). */
public final class Frame {
    public long tMs;
    public Pose2d pose;
    public String autoState;
    public String feedState;
    public boolean followerBusy;
    public double flywheelTargetRadS;
    public double flywheelActualRadS;
    public double turretDeg;
    public double hoodDeg;
    public double intakePower;
}
