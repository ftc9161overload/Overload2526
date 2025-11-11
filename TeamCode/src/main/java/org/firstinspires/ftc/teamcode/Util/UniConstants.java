package org.firstinspires.ftc.teamcode.Util;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//@Config
public class UniConstants {



    //Drive
    public static final String
            DRIVE_FRONT_LEFT_STRING = "LFM", // Exp - 0
            DRIVE_FRONT_RIGHT_STRING = "RFM", // Ctrl - 0
            DRIVE_BACK_LEFT_STRING = "LRM", // Exp - 1
            DRIVE_BACK_RIGHT_STRING = "RRM"; // Ctrl - 1
    public static final String
            DRIVE_FRONT_LEFT_SERVO_STRING = "LFS", // Exp - 0
            DRIVE_FRONT_RIGHT_SERVO_STRING = "RFS",  // Ctrl - 0
            DRIVE_BACK_LEFT_SERVO_STRING = "LRS", // Exp - 1
            DRIVE_BACK_RIGHT_SERVO_STRING = "RRS"; // Ctrl - 1
    public static final String
            DRIVE_FRONT_LEFT_ANALOG_INPUT = "LFA", // Exp - 0
            DRIVE_FRONT_RIGHT_ANALOG_INPUT = "RFA", // Ctrl - 0
            DRIVE_BACK_LEFT_ANALOG_INPUT = "LRA", // Exp  - 1
            DRIVE_BACK_RIGHT_ANALOG_INPUT = "RRA"; // Ctrl - 1
    public static final DcMotorEx.Direction DRIVE_FRONT_LEFT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_FRONT_RIGHT_DIRECTION = DcMotorEx.Direction.REVERSE;
    public static final DcMotorEx.Direction DRIVE_BACK_LEFT_DIRECTION = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction DRIVE_BACK_RIGHT_DIRECTION = DcMotorEx.Direction.REVERSE;


    public static final String
            ROTARY_MOTOR_STRING = "RoM", // Exp - 3
            OUTTAKE_MOTOR_STRING = "OuM", // Exp - 2
            INTAKE_MOTOR_STRING = "InM",// Ctrl - 2
            OUTTAKE_SERVO_STRING = "OuS"; // Exp - 2

    public static final  DcMotorEx.ZeroPowerBehavior ROTARY_ZERO_BEHAVIOR = DcMotorEx.ZeroPowerBehavior.BRAKE, OUTTAKE_ZERO_BEHAVIOR = DcMotorEx.ZeroPowerBehavior.FLOAT, INTAKE_ZERO_BEHAVIOR = DcMotorEx.ZeroPowerBehavior.BRAKE;
    public static final DcMotorEx.RunMode ROTARY_RUN_MODE = DcMotorEx.RunMode.RUN_USING_ENCODER, OUTTAKE_RUN_MODE = DcMotorEx.RunMode.RUN_USING_ENCODER;
    public static final DcMotorEx.Direction ROTARY_DIRECTION = DcMotorEx.Direction.FORWARD, OUTTAKE_DIRECTION = DcMotorEx.Direction.FORWARD, INTAKE_DIRECTION = DcMotorEx.Direction.FORWARD;

    public enum loggingState{
        DISABLED,
        ENABLED,
        EXTREME
    }

    public enum slotState{
        PURPLE,
        GREEN,
        EMPTY
    }

    public static final String COLOR_SENSOR_SLOT_1_STRING = "COLOR0";
    public static final String COLOR_SENSOR_SLOT_2_STRING = "COLOR1";
    public static final String COLOR_SENSOR_SLOT_3_STRING = "COLOR2";




    public static int PURPLE_ARTIFACT_UPPER_HUE = 350;
    public static int PURPLE_ARTIFACT_LOWER_HUE = 275;

    public static int GREEN_ARTIFACT_UPPER_HUE = 150;
    public static int GREEN_ARTIFACT_LOWER_HUE = 100;

    public static final int SPACE_BETWEEN_ROTARY_SLOTS = 300;


//    public static final AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
//            .setDrawTagID(true)
//            .setDrawAxes(true)
//            .setDrawCubeProjection(true)
//            .build();
//    public static final ColorBlobLocatorProcessor colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
//            .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
//            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
//            .setDrawContours(true)   // Show contours on the Stream Preview
//            .setBoxFitColor(0)       // Disable the drawing of rectangles
//            .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
//            .setBlurSize(5)          // Smooth the transitions between different colors in image
//
//            // the following options have been added to fill in perimeter holes.
//            .setDilateSize(15)       // Expand blobs to fill any divots on the edges
//            .setErodeSize(15)        // Shrink blobs back to original size
//            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
//
//            .build();
//
//    public static final ColorBlobLocatorProcessor colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
//            .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
//            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
//            .setDrawContours(true)   // Show contours on the Stream Preview
//            .setBoxFitColor(0)       // Disable the drawing of rectangles
//            .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
//            .setBlurSize(5)          // Smooth the transitions between different colors in image
//
//            // the following options have been added to fill in perimeter holes.
//            .setDilateSize(15)       // Expand blobs to fill any divots on the edges
//            .setErodeSize(15)        // Shrink blobs back to original size
//            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
//
//            .build();
//
//
//
//


}
