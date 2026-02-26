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
    public static Pose2D closeStart = new Pose2D(22.0, 125.0, Math.toRadians(144));

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


}