package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.Vector2D;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;

/**
 * Follower subsystem for autonomous path following and teleop control.
 * Handles coordinate following, heading control, and alliance color mirroring.
 * Integrates with PedroPathing field visualization.
 */
public class Follower implements Subsystem {

    // ========== SINGLETON AND FIELD SETUP ==========

    /** Singleton instance for easy access across OpModes */
    public static final Follower INSTANCE = new Follower();

    /** Field visualization manager for PedroPathing panels */
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    /** FTC field width in inches (12 feet = 144 inches) */
    private static final double FIELD_WIDTH = 144;

    // ========== STATE VARIABLES ==========

    /** Current robot position and orientation */
    private double xPos, yPos, heading;

    /** Target position and orientation for autonomous control */
    private double xTarget, yTarget, headingTarget;

    /** Goal position for scoring [x, y] - used for auto-aiming */
    private double[] goal = {8, 136};

    // ========== CONTROL FLAGS ==========

    /** When true, robot follows the linear path to (xTarget, yTarget) */
    private boolean linearFollower = false;

    /** When true, robot rotates to match headingTarget */
    private boolean headingFollower = false;

    /** When true, driver input is relative to field orientation instead of robot */
    private boolean fieldCentric = true;

    // ========== ALLIANCE COLOR ==========

    /** Enum defining team alliance colors */
    public enum TEAMCOLOR {
        RED,   // Red alliance - robot on red side
        BLUE   // Blue alliance - robot on blue side
    }

    /** Current team color - automatically mirrors coordinates for RED alliance */
    public TEAMCOLOR teamcolor = TEAMCOLOR.BLUE;

    // ========== CONTROLLERS ==========

    /** PID controller for linear (x,y) movement with feedforward */
    private final PDFLController xCon = new PDFLController(0.0, 0, 0, 0.35);

    /** PID controller for rotational movement (heading) with feedforward */
    private final PDFLControllerRadial headingCon = new PDFLControllerRadial(0.0, 0, 0, 0.35);

    // ========== ERROR THRESHOLDS ==========

    /** Minimum linear error (inches) before controller output goes to zero */
    private final double xErrorMin = 0.5;

    /** Minimum heading error (radians) before controller output goes to zero */
    private final double headingErrorMin = 0.1;

    // ========== INITIALIZATION ==========

    /**
     * Initialize the subsystem - sets up field visualization offsets.
     * Called when OpMode initializes.
     */
    public void initialize() {
        // Configure field visualization to match PedroPathing coordinate system
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    // ========== UTILITY METHODS ==========

    /**
     * Normalizes an angle to the range [-π, π].
     * @param a Angle in radians
     * @return Normalized angle in radians
     */
    private double normAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    /**
     * Flips x-coordinate across field centerline (for RED alliance mirroring).
     * @param x X-coordinate in inches
     * @return Mirrored x-coordinate
     */
    private double flipX(double x) {
        return FIELD_WIDTH - x;
    }

    /**
     * Flips an angle across the vertical axis (for RED alliance mirroring).
     * Converts angle to vector, negates y-component, then converts back.
     * @param angle Angle in radians
     * @return Mirrored angle in radians
     */
    private double flipXAngle(double angle) {
        double x = Math.cos(angle);
        double y = Math.sin(angle);
        return new Vector2D(x, -y).angle();
    }

    // ========== POSITION UPDATE ==========

    /**
     * Updates the current robot position and heading.
     * Call this every loop with odometry data.
     * @param xPos Current x position (inches)
     * @param yPos Current y position (inches)
     * @param heading Current heading angle (radians)
     */
    public void update(double xPos, double yPos, double heading) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.heading = heading;
    }

    // ========== RANGE DETECTION COMMANDS ==========

    /**
     * Command that finishes when robot is within range of linear target.
     * Useful for autonomous path following.
     * @param range Maximum distance (inches) to consider "at target"
     * @return Command that completes when within range
     */
    public Command withinRangeLinear(double range) {
        return new LambdaCommand("Follower linear range").setIsDone(() ->
                Math.hypot(xPos - xTarget, yPos - yTarget) < range
        );
    }

    /**
     * Command that finishes when robot heading is within range of target heading.
     * Useful for autonomous rotation control.
     * @param range Maximum angle error (radians) to consider "at target"
     * @return Command that completes when within range
     */
    public Command withinRangeHeading(double range) {
        return new LambdaCommand("Follower heading range").setIsDone(() ->
                Math.abs(normAngle(heading - headingTarget)) < range
        );
    }

    /**
     * Command that finishes when BOTH position and heading are within range.
     * Useful for precise autonomous positioning.
     * @param rangeH Maximum heading error (radians)
     * @param rangeP Maximum position error (inches)
     * @return Command that completes when both criteria met
     */
    public Command withinRange(double rangeH, double rangeP) {
        return new LambdaCommand("Follower both range").setIsDone(() ->
                Math.abs(normAngle(heading - headingTarget)) < rangeH &&
                        Math.hypot(xPos - xTarget, yPos - yTarget) < rangeP
        );
    }

    // ========== TARGET SETTING COMMANDS ==========

    /**
     * Sets target position AND heading (full 3DOF control).
     * Automatically mirrors coordinates for RED alliance.
     * @param xTarget Target x position (inches)
     * @param yTarget Target y position (inches)
     * @param headingTarget Target heading angle (radians)
     * @return InstantCommand that sets the targets immediately
     */
    public Command set(double xTarget, double yTarget, double headingTarget) {
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
     * Sets only linear target position (x,y), leaving heading unchanged.
     * @param xTarget Target x position (inches)
     * @param yTarget Target y position (inches)
     * @return InstantCommand that sets position target
     */
    public Command setLinear(double xTarget, double yTarget) {
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
     * Sets only heading target, leaving position unchanged.
     * @param headingTarget Target heading angle (radians)
     * @return InstantCommand that sets heading target
     */
    public Command setHeading(double headingTarget) {
        return new InstantCommand(() -> {
            if (teamcolor == TEAMCOLOR.RED) {
                this.headingTarget = flipXAngle(headingTarget);
            } else {
                this.headingTarget = headingTarget;
            }
        });
    }

    /**
     * Points robot heading toward the goal position.
     * Calculates angle from current position to goal.
     */
    public void turnToGoal() {
        // Calculate angle from robot to goal
        headingTarget = new Vector2D(goal[0] - xPos, goal[1] - yPos).angle();

        // Mirror for red alliance if needed
        if (teamcolor == TEAMCOLOR.RED) {
            headingTarget = flipXAngle(headingTarget);
        }
    }

    // ========== FOLLOWER CONTROL COMMANDS ==========

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

    // ========== CONTROLLER OUTPUT METHODS ==========

    /**
     * Gets autonomous linear movement vector.
     * Returns PID-controlled velocity toward target when linearFollower is true.
     * @return Vector representing linear movement power/velocity
     */
    public Vector2D getLinear() {
        if (linearFollower) {
            // Update controller with current distance from origin
            xCon.update(Math.hypot(xPos, yPos));

            // Calculate angle toward target
            double angleToTarget = Math.atan2(xTarget - xPos, yTarget - yPos);

            // Return velocity vector pointing toward target
            return new Vector2D(xCon.runPDFL(xErrorMin), 0).rotate(angleToTarget);
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
            headingCon.update(heading);

            // Return angular velocity to reach target heading
            return new Vector2D(headingCon.runPDFL(headingErrorMin), 0);
        }
        return new Vector2D(0, 0);
    }

    // ========== TELEOP CONTROL METHODS ==========

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
            return new Vector2D(x, y).rotate(-heading);
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

    // ========== PERIODIC UPDATE ==========

    /**
     * Called every loop cycle by NextFTC's command scheduler.
     * Updates controllers and field visualization.
     */
    @Override
    public void periodic() {
        // Update controller targets
        headingCon.setTarget(headingTarget);
        xCon.setTarget(Math.hypot(xTarget, yTarget));

        // Update field visualization
        panelsField.moveCursor(xPos, yPos);           // Show current position
        panelsField.setCursorHeading(heading);        // Show current heading
        panelsField.line(xTarget, yTarget);           // Draw line to target
        panelsField.update();                          // Refresh display
    }

    // ========== DEBUG INFORMATION ==========

    /**
     * Generates formatted debug string with all follower state information.
     * Useful for telemetry display during testing.
     * @return Multi-line debug string
     */
    public String debugText() {
        // Calculate current errors
        double linearError = Math.hypot(xPos - xTarget, yPos - yTarget);
        double headingError = normAngle(headingTarget - heading);

        // Format all state information
        return String.format(
                "Follower Debug\n" +
                        "Pos: (%.2f, %.2f)\n" +
                        "Target: (%.2f, %.2f)\n" +
                        "Heading: %.2f rad\n" +
                        "Heading Target: %.2f rad\n" +
                        "Linear Err: %.2f\n" +
                        "Heading Err: %.2f\n" +
                        "Linear Follower: %b\n" +
                        "Heading Follower: %b\n" +
                        "Field Centric: %b\n" +
                        "Team: %s",
                xPos, yPos,
                xTarget, yTarget,
                heading,
                headingTarget,
                linearError,
                headingError,
                linearFollower,
                headingFollower,
                fieldCentric,
                teamcolor
        );
    }
}
