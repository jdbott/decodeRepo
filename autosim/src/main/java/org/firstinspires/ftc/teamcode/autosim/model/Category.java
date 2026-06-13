package org.firstinspires.ftc.teamcode.autosim.model;

/**
 * Action categories. The DECODE set is live; the spares exist so a future season is a
 * config (effect-profile) change in the viewer, not a code change here. The viewer maps
 * each category onto a generic visual primitive via its effect profile.
 */
public enum Category {
    DRIVE, SHOOT, FEED, INTAKE, HOOD, TURRET, WAIT,
    // generic, unused this season
    SLIDE, CLAW, ARM, EXTEND
}
