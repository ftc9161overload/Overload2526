package org.firstinspires.ftc.teamcode.subsystems.LowLevel_General;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Odometry subsystem wrapper around GoBildaPinpointDriver.
 * - Single instance (singleton)
 * - Unified initialization
 * - Fail-fast checks for lifecycle mistakes
 * - Defensive periodic/update
 */
public final class Odometry implements Subsystem {

    private volatile GoBildaPinpointDriver pinpointDriver;

    public static final Odometry INSTANCE = new Odometry();

    private Odometry() {}

    /**
     * Initialize the pinpoint driver with default configuration used in normal operation.
     * Must be called once during robot initialization (OpMode init).
     */
    @Override
    public void initialize() {
        setupPinpoint(true);
    }

    /**
     * Alternate initialization for "real" hardware runs where you may want a lighter setup.
     * Keeps encoder resolution the same but currently does not change directions.
     * Call from the appropriate lifecycle (if used).
     */
    public void initReal() {
        setupPinpoint(false);
    }

    private void setupPinpoint(boolean setDirections) {
        // Acquire hardwareMap from ActiveOpMode (may throw if called too early)
        HardwareMap hw = ActiveOpMode.hardwareMap();
        GoBildaPinpointDriver driver = hw.get(GoBildaPinpointDriver.class, UniConstants.PINPOINT);

        // NOTE: Confirm the API type expected by setEncoderResolution; this mirrors original code.
        driver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        if (setDirections) {
            driver.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.REVERSED,
                    GoBildaPinpointDriver.EncoderDirection.REVERSED
            );
        }

        // publish to volatile field atomically
        this.pinpointDriver = driver;
    }

    /**
     * Create a reset command on demand so it doesn't capture a null driver at class-load time.
     * The command will succeed only if the subsystem has been initialized; otherwise it throws.
     */
    public Command resetCommand() {
        return new InstantCommand(() -> {
            GoBildaPinpointDriver d = checkDriver();
            d.resetPosAndIMU();
        });
    }

    /**
     * Set absolute pose (units must match what Pinpoint expects).
     * x, y in inches; dir in radians (per original code). Verify with your hardware API if needed.
     */
    public void setPos(double x, double y, double dir) {
        GoBildaPinpointDriver d = checkDriver();
        d.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, dir));
    }

    /**
     * Returns the Pose2D from the driver. Throws if not initialized.
     */
    public Pose2D getPos() {
        GoBildaPinpointDriver d = checkDriver();
        return d.getPosition();
    }

    public double getX() {
        GoBildaPinpointDriver d = checkDriver();
        return d.getPosX(DistanceUnit.INCH);
    }

    public double getY() {
        GoBildaPinpointDriver d = checkDriver();
        return d.getPosY(DistanceUnit.INCH);
    }

    public double getHeading() {
        GoBildaPinpointDriver d = checkDriver();
        return d.getHeading(AngleUnit.RADIANS);
    }

    /**
     * Update loop — safe to call even if subsystem hasn't been initialized (no-op).
     * Prefer calling initialize() before OpMode start to enable updates.
     */
    @Override
    public void periodic() {
        GoBildaPinpointDriver d = this.pinpointDriver;
        if (d != null) {
            d.update();
        }
    }

    /**
     * Helper to ensure caller doesn't accidentally operate on a null driver.
     * Throws IllegalStateException with a clear message so lifecycle issues are obvious.
     */
    private GoBildaPinpointDriver checkDriver() {
        GoBildaPinpointDriver d = this.pinpointDriver;
        if (d == null) {
            throw new IllegalStateException("Odometry: pinpointDriver is null — call Odometry.INSTANCE.initialize() (or initReal()) before using this subsystem.");
        }
        return d;
    }
}