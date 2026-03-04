package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.Vector2D;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.LowLevel_General.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.Robot;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.core.units.Distance;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


/**
 * Follower subsystem for autonomous path following and teleop control.
 * Handles coordinate following, heading control, and alliance color mirroring.
 * Integrates with PedroPathing field visualization.
 */
@Configurable
public class Follower implements Subsystem {

    // ========================================================================
    // SINGLETON AND FIELD SETUP
    // ========================================================================

    /** Singleton instance for easy access across OpModes */
    public static final Follower INSTANCE = new Follower();

    /** Field visualization manager for PedroPathing panels */
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    /** FTC field width (12 feet = 144 inches) */
    private static final Distance FIELD_WIDTH = Distance.fromIn(144);


    // ========================================================================
    // STATE VARIABLES
    // ========================================================================

    /** Current robot position and orientation */
    private Pose2D currentPose = new Pose2D(0, 0, 0);

    /**
     * Starting pose offset - subtracted from every odometry update so the robot
     * treats this position as (0, 0, 0). Set via setStartingPose().
     */
    private Pose2D startingPose = new Pose2D(0, 0, 0);

    /** Target position and orientation for autonomous control */
    private Pose2D targetPose = new Pose2D(0, 0, 0);

    /** Goal position for scoring - used for auto-aiming */
    private Pose2D goalPose = new Pose2D(8, 136, 0);


    // ========================================================================
    // VELOCITY VARIABLES
    // ========================================================================

    private Pose2D lastPose = currentPose;

    private Distance xvel = Distance.fromIn(0);
    private Distance yvel = Distance.fromIn(0);

    private Timer timer = new Timer();

    // ========================================================================
    // CONTROL FLAGS
    // ========================================================================

    /** When true, robot follows the linear path to target position */
    private boolean linearFollower = false;

    /** When true, robot rotates to match target heading */
    private boolean headingFollower = false;

    /** When true, driver input is relative to field orientation instead of robot */
    private boolean fieldCentric = true;


    // ========================================================================
    // ALLIANCE COLOR
    // ========================================================================



    /** Current team color - automatically mirrors coordinates for RED alliance */
    public Robot.TEAMCOLOR teamcolor = Robot.TEAMCOLOR.BLUE;


    // ========================================================================
    // CONTROLLERS
    // ========================================================================

    public static double
            pLin = 0.05,//0.18,
            dLin  = 0,//.01,
            fLin = 0,
            lLin = 0.25;

    /** PID controller for linear (x,y) movement with feedforward */
    private final PDFLController linearCon = new PDFLController(0.0, 0, 0, 0);



    /** PID controller for rotational movement (heading) with feedforward */

    public static double
            pHead = 0.4,//0.1,
            dHead,
            fHead,
            lHead = 0.2;
    private final PDFLControllerRadial headingCon = new PDFLControllerRadial(0.0, 0, 0, 0);


    // ========================================================================
    // ERROR THRESHOLDS
    // ========================================================================

    /** Minimum linear error before controller output goes to zero */
    private final Distance linearErrorMin = Distance.fromIn(0.25);

    /** Minimum heading error before controller output goes to zero */
    private final Angle headingErrorMin = Angle.fromRad(0.04);


    // ========================================================================
    // INITIALIZATION
    // ========================================================================

    /**
     * Initialize the subsystem - sets up field visualization offsets.
     * Called when OpMode initializes.
     */
    public void initialize() {
        // Configure field visualization to match PedroPathing coordinate system
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        linearCon.setTarget(0);

    }


    // ========================================================================
    // UTILITY METHODS
    // ========================================================================

    /**
     * Normalizes an angle to the range [-π, π].
     * @param angle Angle to normalize
     * @return Normalized angle
     */
    private Angle normAngle(Angle angle) {
        double a = angle.inRad;
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return Angle.fromRad(a);
    }

    /**
     * Flips x-coordinate across field centerline (for RED alliance mirroring).
     * @param x X-coordinate
     * @return Mirrored x-coordinate
     */
    private Distance flipX(Distance x) {
        return Distance.fromIn(FIELD_WIDTH.inIn - x.inIn);
    }

    /**
     * Flips an angle across the vertical axis (for RED alliance mirroring).
     * Converts angle to vector, negates y-component, then converts back.
     * @param angle Angle to flip
     * @return Mirrored angle
     */
    private Angle flipXAngle(Angle angle) {
        double x = Math.cos(angle.inRad);
        double y = Math.sin(angle.inRad);
        return Angle.fromRad(new Vector2D(x, -y).angle());
    }

    /**
     * Mirrors a pose for RED alliance (flips x-coordinate and heading).
     * @param pose Pose to mirror
     * @return Mirrored pose
     */
    private Pose2D flipPose(Pose2D pose) {
        return new Pose2D(flipX(pose.x), pose.y, flipXAngle(pose.heading));
    }


    // ========================================================================
    // POSITION UPDATE
    // ========================================================================

    /**
     * Updates the current robot pose.
     * Call this every loop with odometry data.
     * @param pose Current robot pose (position + heading)
     */
    public void update(Pose2D pose) {
        this.currentPose = applyStartingOffset(pose);
    }

    /**
     * Updates the current robot position and heading.
     * Call this every loop with odometry data.
     * @param xPos Current x position
     * @param yPos Current y position
     * @param heading Current heading angle
     */
    public void update(Distance xPos, Distance yPos, Angle heading) {
        this.currentPose = applyStartingOffset(new Pose2D(xPos, yPos, heading));
    }

    /**
     * Updates position using raw doubles (inches and radians).
     * Convenience method for compatibility with existing code.
     * @param xPos X position in inches
     * @param yPos Y position in inches
     * @param heading Heading in radians
     */
    public void update(double xPos, double yPos, double heading) {
        this.currentPose = applyStartingOffset(new Pose2D(xPos, yPos, heading));
    }

    /**
     * Applies the starting pose offset to a raw odometry pose.
     * Subtracts the starting position and normalizes the heading delta,
     * so the robot treats startingPose as (0, 0, 0).
     */
    private Pose2D applyStartingOffset(Pose2D raw) {
        Vector2D pose = new Vector2D(raw.x.inIn,raw.y.inIn).rotate(startingPose.heading.inRad + Math.toRadians(0));
        Angle heading = Angle.fromRad(raw.heading.inRad + startingPose.heading.inRad);
        Pose2D retPose = new Pose2D(pose.x+startingPose.x.inIn,pose.y + startingPose.y.inIn, heading.inRad);

        return  (teamcolor == Robot.TEAMCOLOR.BLUE ? retPose : flipPose(retPose));
    }



    /**
     * Sets the starting pose offset. All subsequent update() calls will be
     * measured relative to this pose (i.e., this position becomes the origin).
     * Useful for setting a known starting position at the beginning of an auto.
     * @param pose The pose to treat as (0, 0, 0)
     */
    public Command setStartingPose(Pose2D pose) {
        return new InstantCommand(() -> {startingPose = pose;});
    }

    /**
     * Sets the starting pose using typed Distance and Angle values.
     * @param x Starting x position
     * @param y Starting y position
     * @param heading Starting heading angle
     * @return InstantCommand that sets the starting pose immediately
     */
    public Command setStartingPose(Distance x, Distance y, Angle heading) {
        return setStartingPose(new Pose2D(x, y, heading));
    }

    /**
     * Sets the starting pose using raw doubles (inches and radians).
     * @param xIn Starting x in inches
     * @param yIn Starting y in inches
     * @param headingRad Starting heading in radians
     * @return InstantCommand that sets the starting pose immediately
     */
    public Command setStartingPose(double xIn, double yIn, double headingRad) {
        return setStartingPose(new Pose2D(xIn, yIn, headingRad));
    }

    /**
     * Returns the currently configured starting pose offset.
     * @return Starting pose
     */
    public Pose2D getStartingPose() {
        return startingPose;
    }


    // ========================================================================
    // RANGE DETECTION COMMANDS
    // ========================================================================

    /**
     * Command that finishes when robot is within range of linear target.
     * Useful for autonomous path following.
     * @param range Maximum distance to consider "at target"
     * @return Command that completes when within range
     */
    public Command withinRangeLinear(Distance range) {
        return new LambdaCommand("Follower linear range").setIsDone(() -> {
            double dx = currentPose.x.inIn - targetPose.x.inIn;
            double dy = currentPose.y.inIn - targetPose.y.inIn;
            return Math.hypot(dx, dy) < range.inIn;
        });
    }

    /**
     * Convenience overload accepting range in inches.
     */
    public Command withinRangeLinear(double rangeInches) {
        return withinRangeLinear(Distance.fromIn(rangeInches));
    }

    /**
     * Command that finishes when robot heading is within range of target heading.
     * Useful for autonomous rotation control.
     * @param range Maximum angle error to consider "at target"
     * @return Command that completes when within range
     */
    public Command withinRangeHeading(Angle range) {
        return new LambdaCommand("Follower heading range").setIsDone(() ->
                Math.abs(normAngle(Angle.fromRad(currentPose.heading.inRad - targetPose.heading.inRad)).inRad) < range.inRad
        );
    }

    /**
     * Convenience overload accepting range in radians.
     */
    public Command withinRangeHeading(double rangeRadians) {
        return withinRangeHeading(Angle.fromRad(rangeRadians));
    }

    /**
     * Command that finishes when BOTH position and heading are within range.
     * Useful for precise autonomous positioning.
     * @param rangeH Maximum heading error
     * @param rangeP Maximum position error
     * @return Command that completes when both criteria met
     */
    public Command withinRange(Angle rangeH, Distance rangeP) {
        return new LambdaCommand("Follower both range").setIsDone(() -> {
            double headingError = Math.abs(normAngle(Angle.fromRad(currentPose.heading.inRad - targetPose.heading.inRad)).inRad);
            double dx = currentPose.x.inIn - targetPose.x.inIn;
            double dy = currentPose.y.inIn - targetPose.y.inIn;
            double posError = Math.hypot(dx, dy);
            return headingError < rangeH.inRad && posError < rangeP.inIn;
        });
    }

    /**
     * Convenience overload accepting ranges in radians and inches.
     */
    public Command withinRange(double rangeHRadians, double rangePInches) {
        return withinRange(Angle.fromRad(rangeHRadians), Distance.fromIn(rangePInches));
    }


    // ========================================================================
    // TARGET SETTING COMMANDS (NEW POSE2D METHODS)
    // ========================================================================

    /**
     * Sets target pose (position AND heading) using Pose2D.
     * Automatically mirrors coordinates for RED alliance.
     * @param target Target pose
     * @return InstantCommand that sets the target immediately
     */
    public Command set(Pose2D target) {
        return new InstantCommand(() -> {
            if (teamcolor == Robot.TEAMCOLOR.RED) {
                this.targetPose = flipPose(target);
            } else {
                this.targetPose = target;
            }
        });
    }

    /**
     * Sets target position AND heading (full 3DOF control).
     * Automatically mirrors coordinates for RED alliance.
     * @param xTarget Target x position
     * @param yTarget Target y position
     * @param headingTarget Target heading angle
     * @return InstantCommand that sets the targets immediately
     */
    public Command set(Distance xTarget, Distance yTarget, Angle headingTarget) {
        return set(new Pose2D(xTarget, yTarget, headingTarget));
    }

    /**
     * Convenience overload accepting raw doubles (inches and radians).
     */
    public Command set(double xTarget, double yTarget, double headingTarget) {
        return set(new Pose2D(xTarget, yTarget, headingTarget));
    }

    /**
     * Sets only linear target position (x,y) using Pose2D, leaving heading unchanged.
     * @param target Target pose (only position is used)
     * @return InstantCommand that sets position target
     */
    public Command setLinear(Pose2D target) {
        return new InstantCommand(() -> {
            Pose2D newTarget = target.withHeading(targetPose.heading);
            if (teamcolor == Robot.TEAMCOLOR.RED) {
                this.targetPose = flipPose(newTarget);
            } else {
                this.targetPose = newTarget;
            }
        });
    }

    /**
     * Sets only linear target position (x,y), leaving heading unchanged.
     * @param xTarget Target x position
     * @param yTarget Target y position
     * @return InstantCommand that sets position target
     */
    public Command setLinear(Distance xTarget, Distance yTarget) {
        return setLinear(new Pose2D(xTarget, yTarget));
    }

    /**
     * Convenience overload accepting raw doubles (inches).
     */
    public Command setLinear(double xTarget, double yTarget) {
        return setLinear(new Pose2D(xTarget, yTarget));
    }

    /**
     * Sets only heading target, leaving position unchanged.
     * @param headingTarget Target heading angle
     * @return InstantCommand that sets heading target
     */
    public Command setHeading(Angle headingTarget) {
        return new InstantCommand(() -> {
            Pose2D newTarget = targetPose.withHeading(headingTarget);
            if (teamcolor == Robot.TEAMCOLOR.RED) {
                this.targetPose = flipPose(newTarget);
            } else {
                this.targetPose = newTarget;
            }
        });
    }

    /**
     * Convenience overload accepting raw double (radians).
     */
    public Command setHeading(double headingTarget) {
        return setHeading(Angle.fromRad(headingTarget));
    }

    /**
     * Points robot heading toward the goal position.
     * Calculates angle from current position to goal.
     * @return InstantCommand that updates the heading target immediately
     */
    public Command turnToGoal() {
        return new InstantCommand(() -> {
            // Calculate angle from robot to goal
            double dx = goalPose.x.inIn - currentPose.x.inIn;
            double dy = goalPose.y.inIn - currentPose.y.inIn;
            Angle angleToGoal = Angle.fromRad(new Vector2D(dx, dy).angle());

            // Update target with new heading
            Pose2D newTarget = targetPose.withHeading(angleToGoal);

            // Mirror for red alliance if needed
            if (teamcolor == Robot.TEAMCOLOR.RED) {
                targetPose = flipPose(newTarget);
            } else {
                targetPose = newTarget;
            }
        });
    }

    /**
     * Sets the goal position for auto-aiming using Pose2D.
     * @param goal Goal pose (only position is used)
     * @return InstantCommand that sets the goal immediately
     */
    public Command setGoal(Pose2D goal) {
        return new InstantCommand(() -> this.goalPose = goal);
    }

    /**
     * Sets the goal position for auto-aiming.
     * @param x Goal x position
     * @param y Goal y position
     * @return InstantCommand that sets the goal immediately
     */
    public Command setGoal(Distance x, Distance y) {
        return setGoal(new Pose2D(x, y));
    }

    /**
     * Convenience overload accepting raw doubles (inches).
     * @return InstantCommand that sets the goal immediately
     */
    public Command setGoal(double x, double y) {
        return setGoal(new Pose2D(x, y));
    }


    // ========================================================================
    // FOLLOWER CONTROL COMMANDS
    // ========================================================================

    /** Command to enable autonomous linear following */
    public Command turnOnLinear = new InstantCommand(() -> linearFollower = true);

    /** Command to disable autonomous linear following */
    public Command turnOffLinear = new InstantCommand(() -> linearFollower = false);

    /** Command to enable autonomous heading following */
    public Command turnOnHeading = new InstantCommand(() -> headingFollower = true);

    /** Command to disable autonomous heading following */
    public Command turnOffHeading = new InstantCommand(() -> headingFollower = false);

    /** Command to enable field-centric driving */
    public Command turnOnFieldCentric = new InstantCommand(() -> fieldCentric = true);

    /** Command to disable field-centric driving (robot-centric mode) */
    public Command turnOffFieldCentric = new InstantCommand(() -> fieldCentric = false);


    // ========================================================================
    // CONTROLLER OUTPUT METHODS
    // ========================================================================

    /**
     * Gets autonomous linear movement vector.
     * Returns PID-controlled velocity toward target when linearFollower is true.
     * @return Vector representing linear movement power/velocity
     */
    public Vector2D getLinear() {
        if (linearFollower) {



            // Calculate angle toward target
            double dx = -targetPose.x.inIn + currentPose.x.inIn;
            double dy = -targetPose.y.inIn + currentPose.y.inIn;
            double angleToTarget = Math.atan2(dx, -dy);

            linearCon.update(Math.hypot(dx,dy));
            // Return velocity vector pointing toward target

            return new Vector2D(linearCon.runPDFL(linearErrorMin.inIn), 0).rotate(angleToTarget + Math.toRadians(180) - currentPose.heading.inRad);

        }
        return new Vector2D(0, 0);
    }

    /**
     * Gets autonomous rotational movement vector.
     * Returns PID-controlled angular velocity when headingFollower is true.
     * @return Vector with x-component as angular velocity
     */
    public Vector2D getHeading() {
        if (headingFollower) {
            // Update controller with current heading
            headingCon.update(currentPose.heading.inRad);

            // Return angular velocity to reach target heading
            return new Vector2D(headingCon.runPDFL(headingErrorMin.inRad), 0);
        }
        return new Vector2D(0, 0);
    }


    // ========================================================================
    // TELEOP CONTROL METHODS
    // ========================================================================

    /**
     * Processes driver's linear input for teleop.
     * Either returns autonomous control or driver input (field/robot-centric).
     * @param x Driver's x-axis input (strafe)
     * @param y Driver's y-axis input (forward)
     * @return Movement vector for drivetrain
     */
    public Vector2D getTeleOpLinear(double x, double y) {
        // If autonomous follower is active, ignore driver input
        if (linearFollower) return getLinear();

        // Process driver input
        if (fieldCentric) {
            // Rotate input by -heading to make it field-relative
            return new Vector2D(x, y).rotate(-currentPose.heading.inRad);
        } else {
            // Robot-centric: use input as-is
            return new Vector2D(x, y);
        }
    }

    /**
     * Processes driver's rotational input for teleop.
     * Either returns autonomous control or driver input.
     * @param rotational Driver's rotation input
     * @return Rotation vector for drivetrain
     */
    public Vector2D getTeleOpHeading(double rotational) {
        // If autonomous follower is active, ignore driver input
        if (headingFollower) return getHeading();

        // Use driver's rotation input directly
        return new Vector2D(rotational, 0);
    }


    // ========================================================================
    // GETTER METHODS FOR CURRENT STATE
    // ========================================================================

    /**
     * Gets the current robot pose.
     * @return Current pose (position + heading)
     */
    public Pose2D getCurrentPose() {
        return currentPose;
    }

    /**
     * Gets the current target pose.
     * @return Target pose (position + heading)
     */
    public Pose2D getTargetPose() {
        return targetPose;
    }

    /**
     * Gets the current goal pose.
     * @return Goal pose for auto-aiming
     */
    public Pose2D getGoalPose() {
        return goalPose;
    }

    /**
     * Returns the distance from the robot's current position to the goal position.
     * Automatically mirrors the goal pose when on RED alliance so the comparison
     * occurs in the same coordinate frame as currentPose.
     *
     * @return Distance to goal
     */
    public Distance getDistanceToGoal() {

        // Ensure the goal is in the same frame as currentPose
        Pose2D goal = (teamcolor == Robot.TEAMCOLOR.RED) ? flipPose(goalPose) : goalPose;

        double dx = goal.x.inIn - currentPose.x.inIn;
        double dy = goal.y.inIn - currentPose.y.inIn;

        return Distance.fromIn(Math.hypot(dx, dy));
    }

    // ========================================================================
    // PERIODIC UPDATE
    // ========================================================================

    /**
     * Called every loop cycle by NextFTC's command scheduler.
     * Updates controllers and field visualization.
     */
    @Override
    public void periodic() {
        // Update controller targets
        headingCon.setTarget(targetPose.heading.inRad);
        linearCon.setPDFL(pLin, dLin, fLin, lLin);
        headingCon.setPDFL(pHead, dHead, fHead, lHead);




        // Update field visualization (uses inches for display)
        panelsField.moveCursor(currentPose.x.inIn, currentPose.y.inIn);      // Show current position
        panelsField.setCursorHeading(currentPose.heading.inRad);             // Show current heading
        panelsField.line(targetPose.x.inIn, targetPose.y.inIn);              // Draw line to target
        panelsField.update();                                                // Refresh display

        // Velocity calculation
        double dt = timer.getTimeSeconds();
        xvel = Distance.fromIn((currentPose.x.inIn - lastPose.x.inIn) / dt);
        yvel = Distance.fromIn((currentPose.y.inIn - lastPose.y.inIn) / dt);
        lastPose = currentPose;
        timer.reset();
    }


    // ========================================================================
    // DEBUG INFORMATION
    // ========================================================================

    /**
     * Generates formatted debug string with all follower state information.
     * Useful for telemetry display during testing.
     * @return Multi-line debug string
     */
    public String debugText() {
        // ── Pose errors ──────────────────────────────────────────────────────
        double dx           = targetPose.x.inIn - currentPose.x.inIn;
        double dy           = targetPose.y.inIn - currentPose.y.inIn;
        double linearError  = Math.hypot(dx, dy);
        double headingError = normAngle(Angle.fromRad(targetPose.heading.inRad - currentPose.heading.inRad)).inRad;

        // ── Angle to target ───────────────────────────────────────────────────
        double angleToTargetRad = Math.atan2(dx, dy);
        double angleToTargetDeg = Math.toDegrees(angleToTargetRad);

        // ── Goal geometry ─────────────────────────────────────────────────────
        double gx              = goalPose.x.inIn - currentPose.x.inIn;
        double gy              = goalPose.y.inIn - currentPose.y.inIn;
        double distToGoal      = Math.hypot(gx, gy);
        double angleToGoalRad  = Math.atan2(gx, gy);
        double angleToGoalDeg  = Math.toDegrees(angleToGoalRad);
        double headingGoalErr  = normAngle(Angle.fromRad(angleToGoalRad - currentPose.heading.inRad)).inRad;

        // ── Velocity magnitude ────────────────────────────────────────────────
        double speed = Math.hypot(xvel.inIn, yvel.inIn);

        // ── Controller outputs (call once, reuse) ─────────────────────────────
        Vector2D linearOut  = getLinear();
        Vector2D headingOut = getHeading();
        double   linearMag  = Math.hypot(linearOut.x, linearOut.y);

        // ── At-target flags ───────────────────────────────────────────────────
        boolean atLinearTarget  = linearError  < linearErrorMin.inIn;
        boolean atHeadingTarget = Math.abs(headingError) < headingErrorMin.inRad;

        return String.format(
                        "╔══════════════════════════════════════╗\n"  +
                        "║          FOLLOWER  TELEMETRY         ║\n"  +
                        "╚══════════════════════════════════════╝\n"  +

                        "── POSE ────────────────────────────────\n"  +
                        "  Current  : (%.2f\", %.2f\")  %.2f° (%.4f rad)\n" +
                        "  Target   : (%.2f\", %.2f\")  %.2f° (%.4f rad)\n" +
                        "  Starting : (%.2f\", %.2f\")  %.2f° (%.4f rad)\n" +

                        "── ERRORS ──────────────────────────────\n"  +
                        "  Linear   : %.3f\"  [thresh %.3f\"]  %s\n"  +
                        "  Heading  : %.3f°  (%.4f rad)  [thresh %.3f rad]  %s\n" +
                        "  Angle→Tgt: %.2f°  (%.4f rad)\n"            +

                        "── VELOCITY ────────────────────────────\n"  +
                        "  vX: %.3f in/s   vY: %.3f in/s   |v|: %.3f in/s\n" +

                        "── GOAL ────────────────────────────────\n"  +
                        "  Goal Pose       : (%.2f\", %.2f\")\n"       +
                        "  Dist to Goal    : %.3f\"\n"                 +
                        "  Angle to Goal   : %.2f°  (%.4f rad)\n"     +
                        "  Heading Err→Goal: %.2f°  (%.4f rad)\n"     +

                        "── CONTROLLER GAINS ────────────────────\n"  +
                        "  Linear  P=%.4f  D=%.4f  F=%.4f  L=%.4f\n" +
                        "  Heading P=%.4f  D=%.4f  F=%.4f  L=%.4f\n" +

                        "── OUTPUTS ─────────────────────────────\n"  +
                        "  Linear  : (%.4f, %.4f)  |mag|=%.4f\n"      +
                        "  Linear Normal : (%.4f, %.4f) \n"      +
                        "  Heading : %.4f\n"                           +

                        "── FLAGS ───────────────────────────────\n"  +
                        "  linearFollower=%b  headingFollower=%b\n"    +
                        "  fieldCentric=%b    team=%s\n"               +
                        "  atLinear=%b        atHeading=%b\n",

                // POSE
                currentPose.x.inIn,  currentPose.y.inIn,
                currentPose.heading.inDeg, currentPose.heading.inRad,
                targetPose.x.inIn,   targetPose.y.inIn,
                targetPose.heading.inDeg,  targetPose.heading.inRad,
                startingPose.x.inIn, startingPose.y.inIn,
                startingPose.heading.inDeg, startingPose.heading.inRad,

                // ERRORS
                linearError, linearErrorMin.inIn,
                atLinearTarget ? "IN RANGE" : "OUT",
                Math.toDegrees(headingError), headingError, headingErrorMin.inRad,
                atHeadingTarget ? "IN RANGE" : "OUT",
                angleToTargetDeg, angleToTargetRad,

                // VELOCITY
                xvel.inIn, yvel.inIn, speed,

                // GOAL
                goalPose.x.inIn, goalPose.y.inIn,
                distToGoal,
                angleToGoalDeg, angleToGoalRad,
                Math.toDegrees(headingGoalErr), headingGoalErr,

                // GAINS
                pLin, dLin, fLin, lLin,
                pHead, dHead, fHead, lHead,

                // OUTPUTS
                linearOut.x, linearOut.y, linearMag,
                linearOut.rotate(-currentPose.heading.inRad).x,linearOut.rotate(-currentPose.heading.inRad).y,
                headingOut.x,

                // FLAGS
                linearFollower, headingFollower,
                fieldCentric, teamcolor,
                atLinearTarget, atHeadingTarget
        );
    }
}