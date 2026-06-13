package org.firstinspires.ftc.teamcode.autosim.model;

import org.firstinspires.ftc.teamcode.autosim.geom.Pose2d;

/** Run-level context. */
public final class Meta {
    public String autoName;
    public String alliance = "BLUE";        // trace stored blue-native; viewer mirrors for red
    public int simDtMillis = 20;
    public long totalMillis = 0;            // set in Phase 2
    public String source = "SIM";           // or "REPLAY"
    public String generatedAt;
    public String effectProfile = "decode"; // which category -> effect mapping the viewer applies
    public Pose2d startPose;
    public String followerModel = "pointmass-v1";
    public double followerXVel = 73.8;      // from Pedro Constants
    public double followerYVel = 60.8;
}
