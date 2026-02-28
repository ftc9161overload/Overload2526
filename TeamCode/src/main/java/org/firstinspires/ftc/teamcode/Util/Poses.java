package org.firstinspires.ftc.teamcode.Util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.LowLevel_General.Pose2D;

// Make your poses using THIS website: https://visualizer.pedropathing.com
// All tutorials and explanations can be found HERE: https://pedropathing.com

public class Poses {

    // ── Shared ──────────────────────────────────────────────────────────────
    public static Pose2D start    = new Pose2D(0, 0, 0);
    public static Pose2D blueGoal = new Pose2D(16, 132);

    // ── Close Auton (starting pos: 22, 125, 144°) ───────────────────────────

    // Robot starts here
    public static Pose2D closeStart = new Pose2D(22.0, 125.0, Math.toRadians(143.5));

    // First run: drive to goal, shoot, then pick up ring 1
    public static Pose2D closeGoal       = new Pose2D(45.0, 103.0, Math.toRadians(140));
    public static Pose2D spike1Start    = new Pose2D(45.0,  84.0, Math.toRadians(180));
    public static Pose2D spike1End    = new Pose2D(11.0,  84.0, Math.toRadians(180));

    // Second run: return to goal, shoot, then pick up ring 2
    // (same goal xy, heading unchanged at 140° on arrival)
    public static Pose2D spike2Start    = new Pose2D(45.0,  60.0, Math.toRadians(180));
    public static Pose2D spike2End    = new Pose2D(11.0,  60.0, Math.toRadians(180));

    // Third run: return to goal, shoot, then pick up ring 3
    public static Pose2D spike3Start    = new Pose2D(45.0,  36.0, Math.toRadians(180));
    public static Pose2D spike3End    = new Pose2D(11.0,  36.0, Math.toRadians(180));

    // -*- Common Starts -*-
    public static Pose2D startFar = new Pose2D(56.133, 8, Math.toRadians(90));

    // Goal
    //public static Pose blueGoal = new Pose(16, Math.toRadians(132));
    //public static Pose redGoal = new Pose(128, Math.toRadians(132));

    // Base
    public static Pose2D base = new Pose2D(105.203, 33.319, Math.toRadians(90));
    
    /* Gate
    public static Pose2D gateBluePre = new Pose2D(20.571, Math.toRadians(71.123));
    public static Pose2D gateBlue = new Pose2D(15.628, Math.toRadians(71.123));
    public static Pose2D gateRedPre = new Pose2D(125.502, Math.toRadians(71.123));
    public static Pose2D gateRed = new Pose2D(129.169, Math.toRadians(71.123));
    */
}