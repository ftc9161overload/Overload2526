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

    public static double pLin = 0.0, dLin  = 0, fLin = 0, lLin = 0.35;

    /** PID controller for linear (x,y) movement with feedforward */
    private final PDFLController linearCon = new PDFLController(0.0, 0, 0, 0.35);

    /** PID controller for rotational movement (heading) with feedforward */

    public static double pHead, dHead, fHead, lHead;
    private final PDFLControllerRadial headingCon = new PDFLControllerRadial(0.0, 0, 0, 0.35);


    // ========================================================================
    // ERROR THRESHOLDS
    // ========================================================================

    /** Minimum linear error before controller output goes to zero */
    private final Distance xErrorMin = Distance.fromIn(0.5);

    /** Minimum heading error before controller output goes to zero */
    private final Angle headingErrorMin = Angle.fromRad(0.1);


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
        this.currentPose = pose;
    }

    /**
     * Updates the current robot position and heading.
     * Call this every loop with odometry data.
     * @param xPos Current x position
     * @param yPos Current y position
     * @param heading Current heading angle
     */
    public void update(Distance xPos, Distance yPos, Angle heading) {
        this.currentPose = new Pose2D(xPos, yPos, heading);
    }

    /**
     * Updates position using raw doubles (inches and radians).
     * Convenience method for compatibility with existing code.
     * @param xPos X position in inches
     * @param yPos Y position in inches
     * @param heading Heading in radians
     */
    public void update(double xPos, double yPos, double heading) {
        this.currentPose = new Pose2D(xPos, yPos, heading);
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
     */
    public void turnToGoal() {
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
    }

    /**
     * Sets the goal position for auto-aiming using Pose2D.
     * @param goal Goal pose (only position is used)
     */
    public void setGoal(Pose2D goal) {
        this.goalPose = goal;
    }

    /**
     * Sets the goal position for auto-aiming.
     * @param x Goal x position
     * @param y Goal y position
     */
    public void setGoal(Distance x, Distance y) {
        this.goalPose = new Pose2D(x, y);
    }

    /**
     * Convenience overload accepting raw doubles (inches).
     */
    public void setGoal(double x, double y) {
        this.goalPose = new Pose2D(x, y);
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
            // Calculate current distance from origin
            double distFromOrigin = Math.hypot(currentPose.x.inIn, currentPose.y.inIn);

            // Update controller with current distance
            linearCon.update(distFromOrigin);

            // Calculate angle toward target
            double dx = targetPose.x.inIn - currentPose.x.inIn;
            double dy = targetPose.y.inIn - currentPose.y.inIn;
            double angleToTarget = Math.atan2(dx, dy);

            // Return velocity vector pointing toward target
            return new Vector2D(linearCon.runPDFL(xErrorMin.inIn), 0).rotate(angleToTarget);
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


        double targetDistFromOrigin = Math.hypot(targetPose.x.inIn, targetPose.y.inIn);
        linearCon.setTarget(targetDistFromOrigin);

        // Update field visualization (uses inches for display)
        panelsField.moveCursor(currentPose.x.inIn, currentPose.y.inIn);      // Show current position
        panelsField.setCursorHeading(currentPose.heading.inRad);             // Show current heading
        panelsField.line(targetPose.x.inIn, targetPose.y.inIn);              // Draw line to target
        panelsField.update();                                                // Refresh display

        // Velocity crap
        xvel = Distance.fromIn((currentPose.x.inIn - lastPose.y.inIn)/timer.getTimeSeconds());
        yvel = Distance.fromIn((currentPose.y.inIn - lastPose.y.inIn)/timer.getTimeSeconds());
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
        // Calculate current errors
        double dx = currentPose.x.inIn - targetPose.x.inIn;
        double dy = currentPose.y.inIn - targetPose.y.inIn;
        double linearError = Math.hypot(dx, dy);
        double headingError = normAngle(Angle.fromRad(targetPose.heading.inRad - currentPose.heading.inRad)).inRad;

        // Format all state information
        return String.format(
                "=== Follower Debug ===\n" +
                        "Position: (%.2f\", %.2f\")\n" +
                        "Target: (%.2f\", %.2f\")\n" +
                        "Heading: %.2f° (%.3f rad)\n" +
                        "Target Heading: %.2f° (%.3f rad)\n" +
                        "Linear Error: %.2f\"\n" +
                        "Heading Error: %.2f° (%.3f rad)\n" +
                        "Linear Follower: %b\n" +
                        "Heading Follower: %b\n" +
                        "Field Centric: %b\n" +
                        "Team: %s\n" +
                        "Goal: (%.2f\", %.2f\")",
                currentPose.x.inIn, currentPose.y.inIn,
                targetPose.x.inIn, targetPose.y.inIn,
                currentPose.heading.inDeg, currentPose.heading.inRad,
                targetPose.heading.inDeg, targetPose.heading.inRad,
                linearError,
                Math.toDegrees(headingError), headingError,
                linearFollower,
                headingFollower,
                fieldCentric,
                teamcolor,
                goalPose.x.inIn, goalPose.y.inIn
        );
    }
}