package org.firstinspires.ftc.teamcode.subsystems.UpperLevel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.Util.Vector2D;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.SwervePod;

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

    public static SwervePod frontLeft;
    public static SwervePod frontRight;
    public static SwervePod backLeft;
    public static SwervePod backRight;


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
        SwervePod fr = new SwervePod(
                POD_X_OFFSET.times(-1),  // Front Right: -X (right side)
                POD_Y_OFFSET.times(-1),  // Front Right: -Y (front)
                UniConstants.DRIVE_FRONT_RIGHT_SERVO_STRING,
                UniConstants.DRIVE_FRONT_RIGHT_STRING,
                UniConstants.DRIVE_FRONT_RIGHT_ANALOG_INPUT,
                ActiveOpMode.hardwareMap()
        );

        SwervePod fl = new SwervePod(
                POD_X_OFFSET.times(-1),  // Front Left: -X (left side)
                POD_Y_OFFSET,            // Front Left: +Y (front)
                UniConstants.DRIVE_FRONT_LEFT_SERVO_STRING,
                UniConstants.DRIVE_FRONT_LEFT_STRING,
                UniConstants.DRIVE_FRONT_LEFT_ANALOG_INPUT,
                ActiveOpMode.hardwareMap()
        );

        SwervePod br = new SwervePod(
                POD_X_OFFSET,            // Back Right: +X (right side)
                POD_Y_OFFSET.times(-1),  // Back Right: -Y (back)
                UniConstants.DRIVE_BACK_RIGHT_SERVO_STRING,
                UniConstants.DRIVE_BACK_RIGHT_STRING,
                UniConstants.DRIVE_BACK_RIGHT_ANALOG_INPUT,
                ActiveOpMode.hardwareMap()
        );

        SwervePod bl = new SwervePod(
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

        // Store pods in individual variables for easy access
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
    }


    // ========================================================================
    // CONFIGURATION HELPERS
    // ========================================================================

    /**
     * Configures servo-specific settings (MK version, reversal, PID values).
     */
    private void configureServoSettings(SwervePod fl, SwervePod fr,
                                        SwervePod bl, SwervePod br) {
        // Back Left pod configuration
        bl.setServoReverse(true);
        bl.setServoMKII();
        bl.setPDFL(0.5, 0.005, 0, 0.1);


        // Front Right pod configuration
        fr.setServoMKII();
        fr.setServoReverse(false);
//        fr.setPDFL(0.4, 0.005, 0, 0.2);
        fr.setPDFL(0.5, 0.00, 0, 0.15);
        fr.setErrorMin(0.005);

        // Front Left pod configuration
        fl.setPDFL(0.6, 0.005, 0, 0.15);
        fl.setErrorMin(0.005);

        // Back Right pod configuration
        br.setServoReverse(true);
        br.setPDFL(0.5, 0.005, 0, 0.15);
        br.setErrorMin(0.005);




        // Note: Some pods use MK2 servos with different characteristics
        // Uncomment if BR also needs MK2 settings:
        // br.setServoMKII();
        // br.setServoReverse(true);
    }

    /**
     * Sets the calibration offsets for all servos to account for mounting variations.
     */
    private void configureServoOffsets(SwervePod fl, SwervePod fr,
                                       SwervePod bl, SwervePod br) {
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
        frontLeft.cross();
        frontRight.cross();
        backLeft.cross();
        backRight.cross();
    }

    /**
     * Deactivates cross mode, returning pods to normal driving operation.
     */
    public void uncross() {
        frontLeft.unCross();
        frontRight.unCross();
        backLeft.unCross();
        backRight.unCross();
    }


    // ========================================================================
    // MOTOR CONFIGURATION
    // ========================================================================

    /**
     * Sets all drive motors to brake mode.
     * Motors will actively resist movement when power is zero.
     */
    public void setMotorsToBrake() {
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets all drive motors to float mode.
     * Motors will coast freely when power is zero.
     */
    public void setMotorsToFloat() {
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
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
        frontLeft.setPDFL(p, d, f, l);
        frontRight.setPDFL(p, d, f, l);
        backLeft.setPDFL(p, d, f, l);
        backRight.setPDFL(p, d, f, l);
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


//        if (Math.abs(drive.magnitude()) + Math.abs(rotational.magnitude()) > 1) {
//            double normalizeVal =
//                    Math.max(frontLeft.getResultantVector(drive, rotational).magnitude(),
//                    Math.max(frontRight.getResultantVector(drive, rotational).magnitude(),
//                    Math.max(backLeft.getResultantVector(drive,rotational).magnitude(),
//                            backRight.getResultantVector(drive,rotational).magnitude())));
//
//            Vector2D realDrive = drive.scale(1/normalizeVal);
//            Vector2D realRotational = rotational.scale(1/normalizeVal);
//
//            frontLeft.update(realDrive,realRotational);
//            backLeft.update(realDrive,realRotational);
//            frontRight.update(realDrive,realRotational);
//            backRight.update(realDrive,realRotational);
//
//        } else {
//            frontLeft.update(drive,rotational);
//            backLeft.update(drive,rotational);
//            frontRight.update(drive,rotational);
//            backRight.update(drive,rotational);
//        }


        // Find the maximum magnitude among all wheel vectors
        double maxMagnitude = Math.max(
                frontLeft.getResultantVector(drive, rotational).magnitude(),
                Math.max(frontRight.getResultantVector(drive, rotational).magnitude(),
                        Math.max(backLeft.getResultantVector(drive, rotational).magnitude(),
                                backRight.getResultantVector(drive, rotational).magnitude())));

        // Only normalize if we exceed the limit
        if (maxMagnitude > 1) {
            Vector2D scaledDrive = drive.scale(1 / maxMagnitude);
            Vector2D scaledRotational = rotational.scale(1 / maxMagnitude);

            frontLeft.update(scaledDrive, scaledRotational);
            backLeft.update(scaledDrive, scaledRotational);
            frontRight.update(scaledDrive, scaledRotational);
            backRight.update(scaledDrive, scaledRotational);
        } else {
            frontLeft.update(drive, rotational);
            backLeft.update(drive, rotational);
            frontRight.update(drive, rotational);
            backRight.update(drive, rotational);

        }


    }

    public void runDrive(Vector2D drive, Vector2D rotational, double scaler) {
        runDrive(drive.scale(scaler),rotational.scale(scaler));
    }

    /**
     * Simplified drive method using individual x, y, and rotation values.
     *
     * @param x Forward/backward speed (-1.0 to 1.0)
     * @param y Left/right strafe speed (-1.0 to 1.0)
     * @param rotation Rotation speed (-1.0 to 1.0)
     */
    public void simpleRunDrive(double x, double y, double rotation) {
        frontLeft.update(x, y, rotation);
        frontRight.update(x, y, rotation);
        backLeft.update(x, y, rotation);
        backRight.update(x, y, rotation);
    }

    /**
     * Simplified drive method with movement scaling.
     *
     * @param x Forward/backward speed (-1.0 to 1.0)
     * @param y Left/right strafe speed (-1.0 to 1.0)
     * @param rotation Rotation speed (-1.0 to 1.0)
     * @param scaler Global speed multiplier (0.0 to 1.0)
     */
    public void simpleRunDrive(double x, double y, double rotation, double scaler) {
        frontLeft.update(x * scaler, y * scaler, rotation * scaler);
        frontRight.update(x * scaler, y * scaler, rotation * scaler);
        backLeft.update(x * scaler, y * scaler, rotation * scaler);
        backRight.update(x * scaler, y * scaler, rotation * scaler);
    }

    /**
     * Updates all pods without changing drive commands.
     * Used to maintain servo positioning when not actively driving.
     */
    public void updatePods() {
        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();
    }


    // ========================================================================
    // MANUAL SERVO CONTROL (FOR TESTING)
    // ========================================================================

    /**
     * Stops all servos by setting their power to zero.
     * Useful for emergency stops or testing.
     */
    public void setServoPowZero() {
        frontLeft.setServoPower(0);
        frontRight.setServoPower(0);
        backLeft.setServoPower(0);
        backRight.setServoPower(0);
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
    public SwervePod[] getSwervePods() {
        return new SwervePod[]{frontLeft, frontRight, backLeft, backRight};
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
    public String debugText() {

        return "=== Front Left Pod ===\n" +
                frontLeft.debugText() +
                "\n\n" +
                "=== Front Right Pod ===\n" +
                frontRight.debugText() +
                "\n\n" +
                "=== Back Left Pod ===\n" +
                backLeft.debugText() +
                "\n\n" +
                "=== Back Right Pod ===\n" +
                backRight.debugText() +
                "\n\n";
    }
}