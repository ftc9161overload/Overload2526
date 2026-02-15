package org.firstinspires.ftc.teamcode.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Universal constants for the entire robot.
 *
 * This class serves as a central repository for all hardware names, motor configurations,
 * tuning parameters, and other constants used throughout the codebase.
 *
 * Organization:
 * - Drivetrain configuration (motors, servos, sensors)
 * - Swerve drive tuning parameters
 * - Other subsystem hardware (intake, outtake, rotary)
 * - Vision processing constants
 * - Enums for various states and modes
 */
@Configurable
public class UniConstants {

    // ========================================================================
    // DRIVETRAIN - HARDWARE NAMES
    // ========================================================================

    /**
     * Drive motor hardware names.
     * Naming convention: [L/R][F/R]M = [Left/Right][Front/Rear]Motor
     */
    public static final String DRIVE_FRONT_LEFT_STRING = "LFM";    // Expansion Hub - Port 0
    public static final String DRIVE_FRONT_RIGHT_STRING = "RFM";   // Control Hub - Port 0
    public static final String DRIVE_BACK_LEFT_STRING = "LRM";     // Expansion Hub - Port 1
    public static final String DRIVE_BACK_RIGHT_STRING = "RRM";    // Control Hub - Port 1

    /**
     * Steering servo hardware names.
     * Naming convention: [L/R][F/R]S = [Left/Right][Front/Rear]Servo
     */
    public static final String DRIVE_FRONT_LEFT_SERVO_STRING = "LFS";   // Expansion Hub - Port 0
    public static final String DRIVE_FRONT_RIGHT_SERVO_STRING = "RFS";  // Control Hub - Port 0
    public static final String DRIVE_BACK_LEFT_SERVO_STRING = "LRS";    // Expansion Hub - Port 1
    public static final String DRIVE_BACK_RIGHT_SERVO_STRING = "RRS";   // Control Hub - Port 1

    /**
     * Analog input hardware names (for servo position feedback).
     * Naming convention: [L/R][F/R]A = [Left/Right][Front/Rear]Analog
     */
    public static final String DRIVE_FRONT_LEFT_ANALOG_INPUT = "LFA";   // Expansion Hub - Port 0
    public static final String DRIVE_FRONT_RIGHT_ANALOG_INPUT = "RFA";  // Control Hub - Port 0
    public static final String DRIVE_BACK_LEFT_ANALOG_INPUT = "LRA";    // Expansion Hub - Port 1
    public static final String DRIVE_BACK_RIGHT_ANALOG_INPUT = "RRA";   // Control Hub - Port 1


    // ========================================================================
    // DRIVETRAIN - MOTOR DIRECTIONS
    // ========================================================================

    /**
     * Motor directions to ensure positive power moves the robot forward.
     * These may need adjustment based on motor mounting orientation.
     */
    public static final DcMotorEx.Direction DRIVE_FRONT_LEFT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_FRONT_RIGHT_DIRECTION = DcMotorEx.Direction.REVERSE;
    public static final DcMotorEx.Direction DRIVE_BACK_LEFT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_BACK_RIGHT_DIRECTION = DcMotorEx.Direction.REVERSE;


    // ========================================================================
    // SWERVE DRIVE - TUNING PARAMETERS
    // ========================================================================

    /**
     * Joystick deadzone threshold.
     * Input magnitudes below this value are treated as zero to prevent drift.
     */
    public static double deadzone = 0.01;

    /**
     * Angular deadzone for servo alignment (in radians).
     * The drive motor will only engage when the servo is within this angle of the target.
     * Higher values allow driving with less precise alignment (faster but less accurate).
     */
    public static double radialDeadzone = 0.8;

    /**
     * Motor velocity threshold for detecting pod movement (ticks per second).
     * The flip optimization won't change direction if the motor velocity exceeds this value.
     * Prevents mid-movement direction changes that could cause jerky behavior.
     */
    public static double servoMovementDeadzone = 3;


    // ========================================================================
    // AUTONOMOUS - COORDINATES
    // ========================================================================

    /**
     * Far launch position for blue alliance (x, y in field coordinates).
     * TODO: Verify and update these coordinates based on field measurements.
     */
    public static final int[] far_Launch_Blue = {63, 17};


    // ========================================================================
    // OTHER SUBSYSTEMS - HARDWARE NAMES
    // ========================================================================

    public static final String ROTARY_MOTOR_STRING = "RoM";    // Expansion Hub - Port 3
    public static final String OUTTAKE_MOTOR_STRING = "OuM";   // Expansion Hub - Port 2
    public static final String INTAKE_MOTOR_STRING = "InM";    // Control Hub - Port 2
    public static final String OUTTAKE_SERVO_STRING = "OuS";   // Expansion Hub - Port 2

    public static final String ROTARY_ENCODER = "RoEn";
    public static final String PINPOINT = "pinpoint"; // Expansion Hub - I2C bus 1


    // ========================================================================
    // OTHER SUBSYSTEMS - MOTOR CONFIGURATIONS
    // ========================================================================

    /**
     * Zero power behaviors for each subsystem motor.
     * BRAKE: Motor resists movement when unpowered
     * FLOAT: Motor coasts freely when unpowered
     */
    public static final DcMotorEx.ZeroPowerBehavior ROTARY_ZERO_BEHAVIOR = DcMotorEx.ZeroPowerBehavior.BRAKE;
    public static final DcMotorEx.ZeroPowerBehavior OUTTAKE_ZERO_BEHAVIOR = DcMotorEx.ZeroPowerBehavior.FLOAT;
    public static final DcMotorEx.ZeroPowerBehavior INTAKE_ZERO_BEHAVIOR = DcMotorEx.ZeroPowerBehavior.BRAKE;

    /**
     * Run modes for motors that use encoders.
     */
    public static final DcMotorEx.RunMode ROTARY_RUN_MODE = DcMotorEx.RunMode.RUN_USING_ENCODER;
    public static final DcMotorEx.RunMode OUTTAKE_RUN_MODE = DcMotorEx.RunMode.RUN_USING_ENCODER;

    /**
     * Motor directions for subsystems.
     */
    public static final DcMotorEx.Direction ROTARY_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction OUTTAKE_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction INTAKE_DIRECTION = DcMotorEx.Direction.FORWARD;


    // ========================================================================
    // VISION - COLOR SENSORS
    // ========================================================================

    /**
     * Hardware names for color sensors used in sample detection.
     */
    public static final String COLOR_SENSOR_SLOT_1_STRING = "COLOR0"; // Control Hub - I2C 1 | Right Sensor
    public static final String COLOR_SENSOR_SLOT_2_STRING = "COLOR1"; // Expansion Hub - I2C 0 | Back Sensor
    public static final String COLOR_SENSOR_SLOT_3_STRING = "COLOR2"; // Expansion Hub - I2C 2 | Left Sensor


    // ========================================================================
    // VISION - COLOR DETECTION THRESHOLDS
    // ========================================================================

    /**
     * HSV hue ranges for purple artifact detection.
     * Hue is measured in degrees (0-360).
     */
    public static int PURPLE_ARTIFACT_UPPER_HUE = 350;
    public static int PURPLE_ARTIFACT_LOWER_HUE = 275;

    /**
     * HSV hue ranges for green artifact detection.
     */
    public static int GREEN_ARTIFACT_UPPER_HUE = 150;
    public static int GREEN_ARTIFACT_LOWER_HUE = 100;


    // ========================================================================
    // ROTARY MECHANISM - CONSTANTS
    // ========================================================================

    /**
     * Encoder ticks between each slot in the rotary mechanism.
     * Used for position control and slot indexing.
     */
    public static final int SPACE_BETWEEN_ROTARY_SLOTS = 300;


    // ========================================================================
    // ENUMS - SYSTEM STATES
    // ========================================================================

    /**
     * Engagement levels for various mechanisms.
     */
    public enum engagementLevel {
        FULL_ON,   // Maximum engagement
        ON,        // Normal engagement
        OFF,       // Normal disengagement
        FULL_OFF   // Complete disengagement
    }

    /**
     * Drive modes for swerve drivetrain.
     */
    public enum swerveDriveType {
        DEADZONE,  // Simple mode: only updates servo when joystick exceeds deadzone
        TURN_GO    // Optimized mode: uses flip optimization and waits for alignment
    }

    /**
     * Logging verbosity levels.
     */
    public enum loggingState {
        DISABLED,  // No logging
        ENABLED,   // Standard logging
        EXTREME    // Verbose logging for debugging
    }

    /**
     * States for sample storage slots.
     */
    public enum slotState {
        PURPLE,    // Contains purple sample
        GREEN,     // Contains green sample
        EMPTY      // No sample present
    }


    // ========================================================================
    // VISION PROCESSORS (CURRENTLY DISABLED)
    // ========================================================================

    /*
     * AprilTag and color blob locator processors.
     * These are currently commented out but can be enabled for vision processing.
     *
     * AprilTag Processor:
     * - Detects and tracks AprilTags for localization
     * - Draws tag IDs, axes, and cube projections for debugging
     *
     * Color Blob Locators:
     * - Detect purple and green artifacts on the field
     * - Use morphological operations (dilation/erosion) to clean up detection
     * - Region of Interest (ROI) limits detection area to reduce false positives
     */

//    public static final AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
//            .setDrawTagID(true)
//            .setDrawAxes(true)
//            .setDrawCubeProjection(true)
//            .build();
//
//    public static final ColorBlobLocatorProcessor colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
//            .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
//            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
//            .setDrawContours(true)
//            .setBoxFitColor(0)
//            .setCircleFitColor(Color.rgb(255, 255, 0))
//            .setBlurSize(5)
//
//            // Morphological operations to fill perimeter holes
//            .setDilateSize(15)  // Expand blobs to fill divots
//            .setErodeSize(15)   // Shrink blobs back to original size
//            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
//            .build();
//
//    public static final ColorBlobLocatorProcessor colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
//            .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
//            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
//            .setDrawContours(true)
//            .setBoxFitColor(0)
//            .setCircleFitColor(Color.rgb(255, 255, 0))
//            .setBlurSize(5)
//
//            // Morphological operations to fill perimeter holes
//            .setDilateSize(15)  // Expand blobs to fill divots
//            .setErodeSize(15)   // Shrink blobs back to original size
//            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
//            .build();
}