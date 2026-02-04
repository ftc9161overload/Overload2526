package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.Vector2D;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.core.units.Distance;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;

/**
 * Follower subsystem for autonomous path following and teleop control.
 * Handles coordinate following, heading control, and alliance color mirroring.
 * Integrates with PedroPathing field visualization.
 */
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
    private Distance xPos = Distance.fromIn(0);
    private Distance yPos = Distance.fromIn(0);
    private Angle heading = Angle.fromRad(0);

    /** Target position and orientation for autonomous control */
    private Distance xTarget = Distance.fromIn(0);
    private Distance yTarget = Distance.fromIn(0);
    private Angle headingTarget = Angle.fromRad(0);

    /** Goal position for scoring [x, y] - used for auto-aiming */
    private Distance goalX = Distance.fromIn(8);
    private Distance goalY = Distance.fromIn(136);


    // ========================================================================
    // CONTROL FLAGS
    // ========================================================================

    /** When true, robot follows the linear path to (xTarget, yTarget) */
    private boolean linearFollower = false;

    /** When true, robot rotates to match headingTarget */
    private boolean headingFollower = false;

    /** When true, driver input is relative to field orientation instead of robot */
    private boolean fieldCentric = true;


    // ========================================================================
    // ALLIANCE COLOR
    // ========================================================================

    /** Enum defining team alliance colors */
    public enum TEAMCOLOR {
        RED,   // Red alliance - robot on red side
        BLUE   // Blue alliance - robot on blue side
    }

    /** Current team color - automatically mirrors coordinates for RED alliance */
    public TEAMCOLOR teamcolor = TEAMCOLOR.BLUE;


    // ========================================================================
    // CONTROLLERS
    // ========================================================================

    /** PID controller for linear (x,y) movement with feedforward */
    private final PDFLController xCon = new PDFLController(0.0, 0, 0, 0.35);

    /** PID controller for rotational movement (heading) with feedforward */
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


    // ========================================================================
    // POSITION UPDATE
    // ========================================================================

    /**
     * Updates the current robot position and heading.
     * Call this every loop with odometry data.
     * @param xPos Current x position
     * @param yPos Current y position
     * @param heading Current heading angle
     */
    public void update(Distance xPos, Distance yPos, Angle heading) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.heading = heading;
    }

    /**
     * Updates position using raw doubles (inches and radians).
     * Convenience method for compatibility with existing code.
     * @param xPos X position in inches
     * @param yPos Y position in inches
     * @param heading Heading in radians
     */
    public void update(double xPos, double yPos, double heading) {
        this.xPos = Distance.fromIn(xPos);
        this.yPos = Distance.fromIn(yPos);
        this.heading = Angle.fromRad(heading);
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
            double dx = xPos.inIn - xTarget.inIn;
            double dy = yPos.inIn - yTarget.inIn;
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
                Math.abs(normAngle(Angle.fromRad(heading.inRad - headingTarget.inRad)).inRad) < range.inRad
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
            double headingError = Math.abs(normAngle(Angle.fromRad(heading.inRad - headingTarget.inRad)).inRad);
            double dx = xPos.inIn - xTarget.inIn;
            double dy = yPos.inIn - yTarget.inIn;
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
    // TARGET SETTING COMMANDS
    // ========================================================================

    /**
     * Sets target position AND heading (full 3DOF control).
     * Automatically mirrors coordinates for RED alliance.
     * @param xTarget Target x position
     * @param yTarget Target y position
     * @param headingTarget Target heading angle
     * @return InstantCommand that sets the targets immediately
     */
    public Command set(Distance xTarget, Distance yTarget, Angle headingTarget) {
        return new InstantCommand(() -> {
            if (teamcolor == TEAMCOLOR.RED) {
                // Mirror x-coordinate and heading for red alliance
                this.xTarget = flipX(xTarget);
                this.yTarget = yTarget;
                this.headingTarget = flipXAngle(headingTarget);
            } else {
                // Use coordinates as-is for blue alliance
                this.xTarget = xTarget;
                this.yTarget = yTarget;
                this.headingTarget = headingTarget;
            }
        });
    }

    /**
     * Convenience overload accepting raw doubles (inches and radians).
     */
    public Command set(double xTarget, double yTarget, double headingTarget) {
        return set(Distance.fromIn(xTarget), Distance.fromIn(yTarget), Angle.fromRad(headingTarget));
    }

    /**
     * Sets only linear target position (x,y), leaving heading unchanged.
     * @param xTarget Target x position
     * @param yTarget Target y position
     * @return InstantCommand that sets position target
     */
    public Command setLinear(Distance xTarget, Distance yTarget) {
        return new InstantCommand(() -> {
            if (teamcolor == TEAMCOLOR.RED) {
                this.xTarget = flipX(xTarget);
                this.yTarget = yTarget;
            } else {
                this.xTarget = xTarget;
                this.yTarget = yTarget;
            }
        });
    }

    /**
     * Convenience overload accepting raw doubles (inches).
     */
    public Command setLinear(double xTarget, double yTarget) {
        return setLinear(Distance.fromIn(xTarget), Distance.fromIn(yTarget));
    }

    /**
     * Sets only heading target, leaving position unchanged.
     * @param headingTarget Target heading angle
     * @return InstantCommand that sets heading target
     */
    public Command setHeading(Angle headingTarget) {
        return new InstantCommand(() -> {
            if (teamcolor == TEAMCOLOR.RED) {
                this.headingTarget = flipXAngle(headingTarget);
            } else {
                this.headingTarget = headingTarget;
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
        double dx = goalX.inIn - xPos.inIn;
        double dy = goalY.inIn - yPos.inIn;
        Angle angleToGoal = Angle.fromRad(new Vector2D(dx, dy).angle());

        // Mirror for red alliance if needed
        if (teamcolor == TEAMCOLOR.RED) {
            headingTarget = flipXAngle(angleToGoal);
        } else {
            headingTarget = angleToGoal;
        }
    }

    /**
     * Sets the goal position for auto-aiming.
     * @param x Goal x position
     * @param y Goal y position
     */
    public void setGoal(Distance x, Distance y) {
        this.goalX = x;
        this.goalY = y;
    }

    /**
     * Convenience overload accepting raw doubles (inches).
     */
    public void setGoal(double x, double y) {
        setGoal(Distance.fromIn(x), Distance.fromIn(y));
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
            double distFromOrigin = Math.hypot(xPos.inIn, yPos.inIn);

            // Update controller with current distance
            xCon.update(distFromOrigin);

            // Calculate angle toward target
            double dx = xTarget.inIn - xPos.inIn;
            double dy = yTarget.inIn - yPos.inIn;
            double angleToTarget = Math.atan2(dx, dy);

            // Return velocity vector pointing toward target
            return new Vector2D(xCon.runPDFL(xErrorMin.inIn), 0).rotate(angleToTarget);
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
            headingCon.update(heading.inRad);

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
            return new Vector2D(x, y).rotate(-heading.inRad);
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
    // PERIODIC UPDATE
    // ========================================================================

    /**
     * Called every loop cycle by NextFTC's command scheduler.
     * Updates controllers and field visualization.
     */
    @Override
    public void periodic() {
        // Update controller targets
        headingCon.setTarget(headingTarget.inRad);

        double targetDistFromOrigin = Math.hypot(xTarget.inIn, yTarget.inIn);
        xCon.setTarget(targetDistFromOrigin);

        // Update field visualization (uses inches for display)
        panelsField.moveCursor(xPos.inIn, yPos.inIn);        // Show current position
        panelsField.setCursorHeading(heading.inRad);         // Show current heading
        panelsField.line(xTarget.inIn, yTarget.inIn);        // Draw line to target
        panelsField.update();                                 // Refresh display
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
        double dx = xPos.inIn - xTarget.inIn;
        double dy = yPos.inIn - yTarget.inIn;
        double linearError = Math.hypot(dx, dy);
        double headingError = normAngle(Angle.fromRad(headingTarget.inRad - heading.inRad)).inRad;

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
                xPos.inIn, yPos.inIn,
                xTarget.inIn, yTarget.inIn,
                heading.inDeg, heading.inRad,
                headingTarget.inDeg, headingTarget.inRad,
                linearError,
                Math.toDegrees(headingError), headingError,
                linearFollower,
                headingFollower,
                fieldCentric,
                teamcolor,
                goalX.inIn, goalY.inIn
        );
    }
}