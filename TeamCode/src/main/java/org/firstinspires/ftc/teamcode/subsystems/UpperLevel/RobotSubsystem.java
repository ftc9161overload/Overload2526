package org.firstinspires.ftc.teamcode.subsystems.UpperLevel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LowLevel_General.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Follower;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeFlipperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.RotarySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.SwervePodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.VisionSubsystem;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.core.subsystems.Subsystem;

/**
 * RobotSubsystem - Main robot subsystem container
 * Groups all robot subsystems together using the singleton pattern.
 * This class serves as the central hub for all robot subsystems, manages
 * team color configuration, and provides centralized debug telemetry output.
 *
 * @Configurable annotation allows this class to be configured via FTC Dashboard
 */
@Configurable
public class RobotSubsystem extends SubsystemGroup {

    // Swerve drivetrain subsystem - handles omnidirectional robot movement
    // Initialized first to ensure availability before INSTANCE creation
    public static SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();

    // Singleton instance - ensures only one RobotSubsystem exists throughout the program
    public static final RobotSubsystem INSTANCE = new RobotSubsystem();

    // Telemetry object for displaying debug information to the driver station
    // Initialized in initialize() method when subsystem is set up
    Telemetry telemetry;

    /**
     * Initialization method called when the subsystem is first created
     * Sets up the telemetry object for debug output
     * This is part of the subsystem lifecycle and is called automatically
     */
    @Override
    public void initialize() {
        // Assign telemetry to the singleton instance for use throughout the program
        RobotSubsystem.INSTANCE.telemetry = telemetry;
    }

    /**
     * TEAMCOLOR enum - Represents which alliance the robot is on during a match
     * Used to adjust autonomous routines, field-relative movements, and scoring positions
     */
    public enum TEAMCOLOR {
        RED,   // Red alliance - robot on red side
        BLUE   // Blue alliance - robot on blue side
    }

    // Current team color - defaults to BLUE alliance
    // This should be set before autonomous or teleop based on alliance station
    public static TEAMCOLOR teamColor = TEAMCOLOR.BLUE;

    /**
     * Private constructor - prevents external instantiation (singleton pattern)
     * Registers all subsystem instances with the parent SubsystemGroup for
     * coordinated lifecycle management (periodic updates, initialization, etc.)
     */
    private RobotSubsystem() {
        super(LauncherSubsystem.INSTANCE,
                Odometry.INSTANCE,
                IntakeSubsystem.INSTANCE,
                VisionSubsystem.INSTANCE,
                Follower.INSTANCE);
    }

    /**
     * Sets the team color for the robot and propagates it to dependent subsystems
     * This method should be called during initialization based on the alliance station
     *
     * @param color String representing the team color ("blue" or "Blue" for BLUE, anything else for RED)
     */
    public void setTeamColor(String color) {
        // Check if the color is blue (case-insensitive comparison)
        if (color.equalsIgnoreCase("blue")) {
            teamColor = TEAMCOLOR.BLUE;
            // Update Follower subsystem to use blue alliance coordinates
            Follower.INSTANCE.teamcolor = Follower.TEAMCOLOR.BLUE;
        } else {
            // Default to RED for any other input
            teamColor = TEAMCOLOR.RED;
            // Update Follower subsystem to use red alliance coordinates
            Follower.INSTANCE.teamcolor = Follower.TEAMCOLOR.RED;
        }
    }

    // Flag to enable/disable telemetry debug output - defaults to disabled
    // When true, periodic() method will output diagnostic information
    // These flags are configured via FTC Dashboard using the @Configurable annotation
    public static boolean isTelemetry = false;
    public static boolean rotaryTelemetry = false;
    public static boolean outtakeTelemetry = false;
    public static boolean drivetrainTelemetry = false;
    public static boolean intakeTelemetry = false;

    /**
     * Enables or disables telemetry debug output
     *
     * @param state true to enable telemetry output, false to disable
     */
    public void setIsTelemetry(boolean state) {
        isTelemetry = state;
    }

    /**
     * Periodic method called continuously during OpMode execution
     * Handles debug telemetry output when enabled
     * This is part of the subsystem lifecycle and runs every loop iteration
     */
    @Override
    public void periodic() {
        // Only output debug info if telemetry is enabled
        if(isTelemetry) {
            // Enable vision subsystem debug visualization
            VisionSubsystem.INSTANCE.setDebugMode(true);
            // Display follower subsystem debug information to driver station
            telemetry.addLine(Follower.INSTANCE.debugText());
        }

        // Display rotary subsystem telemetry if enabled
        if(rotaryTelemetry) {
            telemetry.addLine(RotarySubsystem.INSTANCE.debugText());
        }

        // Display outtake subsystem telemetry if enabled
        if(outtakeTelemetry) {
            telemetry.addLine(OuttakeFlipperSubsystem.INSTANCE.debugText());
            telemetry.addLine(OuttakeWheelSubsystem.INSTANCE.debugText());
        }

        // Display drivetrain subsystem telemetry if enabled
        if(drivetrainTelemetry) {
            telemetry.addLine(swerveDrivetrain.debugText());
        }

        // Display intake subsystem telemetry if enabled
        if(intakeTelemetry) {
//            telemetry.addLine(IntakeSubsystem.INSTANCE.debugText());
        }
    }
}