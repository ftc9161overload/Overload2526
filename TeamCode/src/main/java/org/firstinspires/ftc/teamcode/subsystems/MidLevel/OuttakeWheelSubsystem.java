package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Util.Lerp;
import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
/**
 * OuttakeWheelSubsystem controls a flywheel motor for shooting game elements.
 * Uses PDFL (PID with FeedForward and Lerp) control for precise velocity management.
 * Supports multiple preset speeds and manual speed adjustment.
 */
public class OuttakeWheelSubsystem implements Subsystem {

    // ========== SINGLETON PATTERN ==========

    /** Singleton instance for easy access across OpModes */
    public static final OuttakeWheelSubsystem INSTANCE = new OuttakeWheelSubsystem();

    // ========== HARDWARE ==========

    /** Flywheel motor - connected via hardware map string constant */
    private final MotorEx motor = new MotorEx(UniConstants.OUTTAKE_MOTOR_STRING);

    // ========== SPEED PRESETS ==========

    /** Predefined target speeds (RPM) for different shot distances */
    public final int[] targetSpeeds = {
            1800,  // Speed preset 1 (close range)
            2200,  // Speed preset 2 (medium range)
            2600   // Speed preset 3 (long range)
    };

    /** Minimum allowed flywheel speed (RPM) */
    private static final int MIN_SPEED = 1600;

    /** Maximum allowed flywheel speed (RPM) */
    private static final int MAX_SPEED = 2600;

    /** Speed increment/decrement for manual adjustment (RPM) */
    private static final int SPEED_STEP = 200;

    /** Tolerance for considering flywheel "at speed" (RPM) */
    private static final double SPEED_TOLERANCE = 60;

    /** Conversion factor: RPM to motor power (empirically determined) */
    private static final double RPM_TO_POWER = 0.00039411;

    // ========== STATE VARIABLES ==========

    /** Target speed the flywheel is trying to reach (RPM) */
    public int targetSpeed = 0;

    /** Current measured flywheel velocity (RPM) */
    private double currentSpeed = 0;

    /** Current motor power being applied [0.0 to 1.0] */
    private double power = 0;

    // ========== CONTROLLERS ==========

    /** Lerp (linear interpolation) for smooth power ramping */
    public Lerp lerp = new Lerp(0, 0, 0);

    /** PDFL controller for velocity correction and stability */
    private final PDFLController pdfl = new PDFLController(0.01, 0, 0.0, 0.000);

    // ========== CONSTRUCTOR ==========

    /** Private constructor enforces singleton pattern */
    private OuttakeWheelSubsystem() {}

    // ========== INITIALIZATION ==========

    /**
     * Initialize the subsystem.
     * Called when OpMode initializes.
     */
    public void initialize() {
        // Optional PDFL error power tuning (currently disabled)
        // pdfl.setErrorPower(1.4);
    }

    // ========== SPEED CHECKING ==========

    /**
     * Checks if flywheel is within acceptable range of target speed.
     * @return true if current speed is within SPEED_TOLERANCE of target
     */
    public boolean withinRangeBool() {
        return Math.abs(targetSpeed - currentSpeed) < SPEED_TOLERANCE;
    }

    /**
     * Command that completes when flywheel reaches target speed.
     * Useful for ensuring flywheel is spun up before shooting.
     * @return Command that finishes when flywheel is at speed
     */
    public Command withinRange() {
        return new LambdaCommand("Flywheel within range?")
                .setIsDone(this::withinRangeBool);
    }

    // ========== PRESET SPEED COMMANDS ==========

    /**
     * Sets flywheel to speed preset 1 (1800 RPM - close range).
     * InstantCommand executes immediately and completes in one cycle.
     */
    public Command setSpeed1 = new InstantCommand(() -> {
        targetSpeed = targetSpeeds[0];
    });

    /**
     * Sets flywheel to speed preset 2 (2200 RPM - medium range).
     */
    public Command setSpeed2 = new InstantCommand(() -> {
        targetSpeed = targetSpeeds[1];
    });

    /**
     * Sets flywheel to speed preset 3 (2600 RPM - long range).
     */
    public Command setSpeed3 = new InstantCommand(() -> {
        targetSpeed = targetSpeeds[2];
    });

    // ========== MANUAL SPEED ADJUSTMENT COMMANDS ==========

    /**
     * Increases target speed by 200 RPM.
     * Clamps result to valid range [1600, 2600].
     */
    public Command setSpeedHigher = new InstantCommand(() -> {
        targetSpeed += SPEED_STEP;
        targetSpeed = Math.min(Math.max(targetSpeed, MIN_SPEED), MAX_SPEED);
    });

    /**
     * Decreases target speed by 200 RPM.
     * Clamps result to valid range [1600, 2600].
     */
    public Command setSpeedLower = new InstantCommand(() -> {
        targetSpeed -= SPEED_STEP;
        targetSpeed = Math.min(Math.max(targetSpeed, MIN_SPEED), MAX_SPEED);
    });

    // ========== STOP COMMAND ==========

    /**
     * Stops the flywheel by setting target speed to 0.
     * Motor will gradually spin down via lerp control.
     */
    public Command turnOff = new InstantCommand(() -> {
        targetSpeed = 0;
    });

    // ========== PERIODIC UPDATE ==========

    /**
     * Called every loop cycle by NextFTC's command scheduler.
     * Handles velocity control using lerp + PDFL correction.
     */
    @Override
    public void periodic() {
        // Read current motor velocity from encoder
        currentSpeed = motor.getVelocity();

        // Update PDFL controller with new target and current velocity
        pdfl.setTarget(targetSpeed);
        pdfl.update(currentSpeed);

        // Lerp smoothly ramps base power toward feedforward estimate
        // Converts target RPM to approximate motor power using constant
        double targetPower = targetSpeed * RPM_TO_POWER;
        power += lerp.constantLerp(power, targetPower, 1);

        // Safety check: prevent NaN from propagating to motor
        if (Double.isNaN(power)) {
            power = 0;
        }

        // Clamp base power to valid motor range [0.0, 1.0]
        power = Math.max(0.0, Math.min(1.0, power));

        // Apply base power + PDFL correction to motor
        // PDFL adds small correction to minimize velocity error
        motor.setPower(power + pdfl.runPDFL(10));
    }

    // ========== DEBUG INFORMATION ==========

    /**
     * Returns compact debug telemetry information for the outtake wheel subsystem.
     * Designed for single-line display in RobotSubsystem periodic telemetry.
     * @return Formatted string with velocity, target, power, and status
     */
    @SuppressLint("DefaultLocale")
    public String debugText() {
        return String.format("Outtake Wheel | Velocity: %.0f RPM | Target: %d RPM | Power: %.3f | At Speed: %b",
                currentSpeed,
                targetSpeed,
                power,
                withinRangeBool());
    }

    /**
     * Generates formatted debug string with all subsystem state information.
     * Useful for telemetry display during testing and tuning.
     * @return Multi-line debug string with velocity, target, and control info
     */
    @SuppressLint("DefaultLocale")
    public String debugString() {
        return String.format(
                "\n=== Outtake Wheel Debug ===" +
                        "\nCurrent Velocity: %.0f RPM" +
                        "\nTarget Speed: %d RPM" +
                        "\nBase Power: %.3f" +
                        "\nLerp Output: %.3f" +
                        "\nAt Speed: %b",
                currentSpeed,
                targetSpeed,
                power,
                lerp.constantLerp(power, targetSpeed * RPM_TO_POWER, 1),
                withinRangeBool()
        );
    }
}