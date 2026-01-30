package org.firstinspires.ftc.teamcode.subsystems.UpperLevel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.Util.Vector2D;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.SwervePodSubsystem;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Distance;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Upper-level subsystem that manages all four swerve pods as a cohesive drivetrain.
 *
 * Handles:
 * - Initialization and configuration of all four pods
 * - Coordinated movement commands across all pods
 * - Cross-mode for defensive positioning
 * - Debug telemetry aggregation
 */
@Configurable
public class SwerveDrivetrain implements Subsystem {

    // ========================================================================
    // SERVO OFFSET CALIBRATION VALUES (in degrees)
    // ========================================================================
    // These offsets compensate for mechanical variations in servo mounting

    public static int blOffset = 45;      // Back Left servo offset
    public static int brOffset = -155;    // Back Right servo offset
    public static int flOffset = 0;       // Front Left servo offset
    public static int frOffset = 140;     // Front Right servo offset


    // ========================================================================
    // POD GEOMETRY CONSTANTS
    // ========================================================================
    // Distance from robot center to each wheel (in millimeters)

    private static final Distance POD_X_OFFSET = Distance.fromMm(156.0);
    private static final Distance POD_Y_OFFSET = Distance.fromMm(156.0);


    // ========================================================================
    // SWERVE PODS
    // ========================================================================

    private final SwervePodSubsystem[] pods;  // Array of all four pods [FL, FR, BL, BR]


    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Initializes the swerve drivetrain with all four pods.
     * <p>
     * Pod layout (looking down at robot):
     * FRONT
     * FL     FR
     * <p>
     * BL     BR
     * BACK
     */
    public SwerveDrivetrain() {
        // Initialize all four swerve pods with their positions and hardware names
        SwervePodSubsystem fr = new SwervePodSubsystem(
                POD_X_OFFSET.times(-1),  // Front Right: -X (right side)
                POD_Y_OFFSET.times(-1),  // Front Right: -Y (front)
                UniConstants.DRIVE_FRONT_RIGHT_SERVO_STRING,
                UniConstants.DRIVE_FRONT_RIGHT_STRING,
                UniConstants.DRIVE_FRONT_RIGHT_ANALOG_INPUT,
                ActiveOpMode.hardwareMap()
        );

        SwervePodSubsystem fl = new SwervePodSubsystem(
                POD_X_OFFSET.times(-1),  // Front Left: -X (left side)
                POD_Y_OFFSET,            // Front Left: +Y (front)
                UniConstants.DRIVE_FRONT_LEFT_SERVO_STRING,
                UniConstants.DRIVE_FRONT_LEFT_STRING,
                UniConstants.DRIVE_FRONT_LEFT_ANALOG_INPUT,
                ActiveOpMode.hardwareMap()
        );

        SwervePodSubsystem br = new SwervePodSubsystem(
                POD_X_OFFSET,            // Back Right: +X (right side)
                POD_Y_OFFSET.times(-1),  // Back Right: -Y (back)
                UniConstants.DRIVE_BACK_RIGHT_SERVO_STRING,
                UniConstants.DRIVE_BACK_RIGHT_STRING,
                UniConstants.DRIVE_BACK_RIGHT_ANALOG_INPUT,
                ActiveOpMode.hardwareMap()
        );

        SwervePodSubsystem bl = new SwervePodSubsystem(
                POD_X_OFFSET,            // Back Left: +X (left side)
                POD_Y_OFFSET,            // Back Left: +Y (back)
                UniConstants.DRIVE_BACK_LEFT_SERVO_STRING,
                UniConstants.DRIVE_BACK_LEFT_STRING,
                UniConstants.DRIVE_BACK_LEFT_ANALOG_INPUT,
                ActiveOpMode.hardwareMap()
        );

        // Configure servo characteristics for each pod
        configureServoSettings(fl, fr, bl, br);

        // Set calibration offsets for each servo
        configureServoOffsets(fl, fr, bl, br);

        // Store pods in array for easy iteration
        // Order: [FL, FR, BL, BR]
        pods = new SwervePodSubsystem[]{fl, fr, bl, br};
    }


    // ========================================================================
    // CONFIGURATION HELPERS
    // ========================================================================

    /**
     * Configures servo-specific settings (MK version, reversal, PID values).
     */
    private void configureServoSettings(SwervePodSubsystem fl, SwervePodSubsystem fr,
                                        SwervePodSubsystem bl, SwervePodSubsystem br) {
        // Back Left pod configuration
        bl.setServoReverse(true);
        bl.setServoMKII();
        bl.setPDFL(0.5, 0.005, 0, 0.1);

        // Front Right pod configuration
        fr.setServoMKII();
        fr.setServoReverse(true);
        fr.setPDFL(0.4, 0.005, 0, 0.1);

        // Front Left pod configuration
        fl.setPDFL(0.4, 0.005, 0, 0.1);

        // Back Right pod configuration
        br.setPDFL(0.3, 0.005, 0, 0.1);

        // Note: Some pods use MK2 servos with different characteristics
        // Uncomment if BR also needs MK2 settings:
        // br.setServoMKII();
        // br.setServoReverse(true);
    }

    /**
     * Sets the calibration offsets for all servos to account for mounting variations.
     */
    private void configureServoOffsets(SwervePodSubsystem fl, SwervePodSubsystem fr,
                                       SwervePodSubsystem bl, SwervePodSubsystem br) {
        fl.setServoOffsetDeg(flOffset);
        fr.setServoOffsetDeg(frOffset);
        bl.setServoOffsetDeg(blOffset);
        br.setServoOffsetDeg(brOffset);
    }


    // ========================================================================
    // CROSS MODE (DEFENSIVE POSITIONING)
    // ========================================================================

    /**
     * Activates cross mode on all pods.
     * In cross mode, pods rotate to form an X pattern, making the robot
     * harder to push and more stable when defending.
     */
    public void cross() {
        for (SwervePodSubsystem pod : pods) {
            pod.cross();
        }
    }

    /**
     * Deactivates cross mode, returning pods to normal driving operation.
     */
    public void uncross() {
        for (SwervePodSubsystem pod : pods) {
            pod.unCross();
        }
    }


    // ========================================================================
    // MOTOR CONFIGURATION
    // ========================================================================

    /**
     * Sets all drive motors to brake mode.
     * Motors will actively resist movement when power is zero.
     */
    public void setMotorsToBrake() {
        for (SwervePodSubsystem pod : pods) {
            pod.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Sets all drive motors to float mode.
     * Motors will coast freely when power is zero.
     */
    public void setMotorsToFloat() {
        for (SwervePodSubsystem pod : pods) {
            pod.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }
    }


    // ========================================================================
    // PID TUNING
    // ========================================================================

    /**
     * Sets PDFL controller values for all pods simultaneously.
     * Useful for bulk tuning during testing.
     *
     * @param p Proportional gain
     * @param d Derivative gain
     * @param f Feedforward gain
     * @param l Limiting factor
     */
    public void setPDFLs(double p, double d, double f, double l) {
        for (SwervePodSubsystem pod : pods) {
            pod.setPDFL(p, d, f, l);
        }
    }


    // ========================================================================
    // DRIVING METHODS
    // ========================================================================

    /**
     * Drives the robot using separate translational and rotational vectors.
     * This is the primary driving method for teleop and autonomous.
     *
     * @param drive Translational velocity vector (x, y components)
     * @param rotational Rotational velocity vector (magnitude determines rotation speed)
     */
    public void runDrive(Vector2D drive, Vector2D rotational) {
        for (SwervePodSubsystem pod : pods) {
            pod.update(drive, rotational);
        }
    }

    /**
     * Simplified drive method using individual x, y, and rotation values.
     *
     * @param x Forward/backward speed (-1.0 to 1.0)
     * @param y Left/right strafe speed (-1.0 to 1.0)
     * @param rotation Rotation speed (-1.0 to 1.0)
     */
    public void simpleRunDrive(double x, double y, double rotation) {
        for (SwervePodSubsystem pod : pods) {
            pod.update(x, y, rotation);
        }
    }

    /**
     * Simplified drive method with movement scaling.
     *
     * @param x Forward/backward speed (-1.0 to 1.0)
     * @param y Left/right strafe speed (-1.0 to 1.0)
     * @param rotation Rotation speed (-1.0 to 1.0)
     * @param movementScaler Global speed multiplier (0.0 to 1.0)
     */
    public void simpleRunDrive(double x, double y, double rotation, double movementScaler) {
        for (SwervePodSubsystem pod : pods) {
            pod.update(x, y, rotation, movementScaler);
        }
    }

    /**
     * Updates all pods without changing drive commands.
     * Used to maintain servo positioning when not actively driving.
     */
    public void updatePods() {
        for (SwervePodSubsystem pod : pods) {
            pod.update();
        }
    }


    // ========================================================================
    // MANUAL SERVO CONTROL (FOR TESTING)
    // ========================================================================

    /**
     * Stops all servos by setting their power to zero.
     * Useful for emergency stops or testing.
     */
    public void setServoPowZero() {
        for (SwervePodSubsystem pod : pods) {
            pod.setServoPower(0);
        }
    }

    // Commented out methods - uncomment if needed for testing

    /**
     * Sets all pods to point at angle zero.
     * Useful for calibration and testing.
     */
    // public void setPosZero() {
    //     for (SwervePodSubsystem pod : pods) {
    //         pod.setPos(Angle.fromRad(0));
    //     }
    // }

    /**
     * Sets all pods to point at a specific angle.
     *
     * @param pos Target angle for all pods
     */
    // public void setPos(Angle pos) {
    //     for (SwervePodSubsystem pod : pods) {
    //         pod.setPos(pos);
    //     }
    // }


    // ========================================================================
    // ACCESSORS
    // ========================================================================

    /**
     * Returns the array of all swerve pods.
     * Useful for direct access to individual pods.
     *
     * @return Array of pods in order: [FL, FR, BL, BR]
     */
    public SwervePodSubsystem[] getSwervePods() {
        return pods;
    }


    // ========================================================================
    // TELEOP INITIALIZATION (PLACEHOLDER)
    // ========================================================================

    /**
     * Initializes drivetrain for teleop mode.
     * Currently a placeholder - add initialization logic as needed.
     */
    public void startTeleopDrive() {
        // TODO: Add any teleop-specific initialization
    }

    /**
     * Initializes drivetrain for teleop mode with optional brake mode.
     *
     * @param brakeMode If true, sets motors to brake; otherwise sets to float
     */
    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) {
            setMotorsToBrake();
        } else {
            setMotorsToFloat();
        }
        // TODO: Add any additional teleop-specific initialization
    }


    // ========================================================================
    // DEBUG / TELEMETRY
    // ========================================================================

    /**
     * Generates a comprehensive debug string with information from all pods.
     *
     * @return Multi-line string containing debug info for each pod
     */
    public String debugString() {
        StringBuilder returnStr = new StringBuilder();

        String[] podNames = {"Front Left", "Front Right", "Back Left", "Back Right"};

        for (int i = 0; i < pods.length; i++) {
            returnStr.append("=== ").append(podNames[i]).append(" Pod ===\n");
            returnStr.append(pods[i].debugText());
            returnStr.append("\n\n");
        }

        return returnStr.toString();
    }
}