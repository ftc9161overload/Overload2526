package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.MathUtil;
import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.core.commands.Command;

@Configurable
public class RotarySubsystem implements Subsystem {
    // ========== SINGLETON PATTERN ==========

    /**
     * Singleton instance for easy access across OpModes
     */
    public static final RotarySubsystem INSTANCE = new RotarySubsystem();

    // ========== HARDWARE ==========

    /**
     * Main rotation motor - configured with brake mode and zeroed position
     */
    private final MotorEx motor = new MotorEx(UniConstants.ROTARY_MOTOR_STRING).brakeMode().zeroed();

    /**
     * Separate encoder for position tracking
     */
    private final MotorEx Encoder = new MotorEx(UniConstants.ROTARY_ENCODER);

    /**
     * Color sensors for detecting ball type in each chamber (3 sensors)
     */
    private final NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[3];

    /**
     * Distance sensor for homing routine - detects wall proximity
     */
    private ColorRangeSensor distSensor;

    // ========== PHYSICAL CONSTANTS ==========

    /**
     * Encoder ticks per full 360° rotation (gear ratio: 170:32)
     */
    private static final double ticksPerRotation = 8192 * 170.0 / 32.0;

    /**
     * Tolerance for determining if position is "close enough" (radians)
     */
    private static final double positionTolerance = 0.02;

    /**
     * Minimum power threshold to consider rotary "stopped" for color reading
     */
    private static final double stoppedPowerThreshold = 0.04;

    // ========== CONTROLLER TUNING ==========

    /**
     * Proportional gain for PDFL controller
     */
    private static final double p = 0.85;

    /**
     * Derivative gain for PDFL controller
     */
    private static final double d = 0.01;

    /**
     * Feedforward gain for PDFL controller
     */
    private static final double f = 0.0;

    /**
     * Lerp (smoothing) factor for PDFL controller
     */
    private static final double l = 0.12;

    /**
     * Current feedforward value (modified by lock/unlock)
     */
    private double fn = f;

    /**
     * PDFL controller for radial (angular) position control
     */
    private final PDFLControllerRadial mCon = new PDFLControllerRadial(p, d, fn, l);

    // ========== COLOR DETECTION CONSTANTS ==========

    /**
     * RGB values for GREEN ball detection (normalized 0-1 scale)
     */
    public static final double[] greenColor = {0.009, 0.04, 0.028};

    // The lower and upper sections of green colors
    public static double[] lowerGreenColors = {0.004, 0.00, 0.023};
    public static double[] higherGreenColors = {0.014, 0.09, 0.033};

    /**
     * RGB values for PURPLE ball detection (normalized 0-1 scale)
     */

    // The lower and upper sections of purple colors
    public static final double[] purpleColor = {0.007, 0.009, 0.014};
    public static double[] lowerPurpleColors = {0.002, 0.004, 0.009};
    public static double[] higherPurpleColors = {0.012, 0.014, 0.019};

    /**
     * Tolerance for color matching (±this amount per channel)
     */
    private static final double colorTolerance = 0.001;

    // ========== STATE VARIABLES ==========

    /**
     * Current chamber being accessed (1, 2, or 3)
     */
    private int currentChamber = 1;

    /**
     * Current rotary position in radians
     */
    private double currentPosition = 0;

    /**
     * Target rotary position in radians
     */
    private double targetPosition = 0;

    /**
     * Calibration offset from homing routine (radians)
     */
    private double offset = 0;

    /**
     * When true, motors are locked (feedforward disabled)
     */
    public boolean locked = true;

    /**
     * When true, position halfway between chambers for loading
     */
    public boolean halfChamber = false;

    /**
     * Angular offset when halfChamber mode is active (radians)
     */
    private double halfOffset = 0;

    /**
     * When true, color sensors will update on next stop
     */
    private boolean shouldUpdateColors = true;

    // ========== HOMING STATE VARIABLES ==========

    /**
     * True during wall-finding phase of homing
     */
    private boolean findWall = false;

    /**
     * True during edge-finding phase of homing
     */
    private boolean findEdge = false;

    /**
     * True after homing is complete and offset is set
     */
    private boolean homingDone = false;

    /**
     * Cached distance sensor reading (inches)
     */
    private double distSensorOutput;

    // ========== BALL TYPE ENUM ==========

    /**
     * Enum representing the type of ball in a chamber.
     * Detected via color sensors.
     */
    public enum Ball {
        PURPLE,  // Purple/red alliance ball
        GREEN,   // Green/blue alliance ball
        NULL     // No ball or unrecognized color
    }

    // ========== CHAMBER ENUM ==========

    /**
     * Enum representing the three chambers of the rotary mechanism.
     * Each chamber has a fixed angle and tracks its current ball type.
     */
    public enum Chamber {
        ONE(0, Ball.NULL),                      // Chamber 1 at 0°
        TWO(2 * Math.PI / 3, Ball.NULL),       // Chamber 2 at 120°
        THREE(4 * Math.PI / 3, Ball.NULL);     // Chamber 3 at 240°

        /**
         * Fixed angular position of this chamber (radians)
         */
        public final double angle;

        /**
         * Type of ball currently in this chamber
         */
        public Ball ball;

        Chamber(double angle, Ball ball) {
            this.angle = angle;
            this.ball = ball;
        }
    }

    /**
     * Array for easy iteration over all chambers
     */
    private static final Chamber[] CHAMBERS = {
            Chamber.ONE,
            Chamber.TWO,
            Chamber.THREE
    };

    // ========== CONSTRUCTOR ==========

    /**
     * Private constructor enforces singleton pattern
     */
    private RotarySubsystem() {
    }

    // ========== INITIALIZATION ==========

    /**
     * Initialize the subsystem.
     * Resets encoder, configures controller, and initializes sensors.
     */
    public void initialize() {
        // Configure PDFL controller with tuning constants
        mCon.setPDFL(p, d, fn, l);

        // Reset and configure encoder
        motor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize color sensors from hardware map
        colorSensors[0] = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_1_STRING);
        colorSensors[1] = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_2_STRING);
        colorSensors[2] = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_3_STRING);

        // Initialize distance sensor (same hardware as cs2, different interface)
        distSensor = ActiveOpMode.hardwareMap().get(ColorRangeSensor.class, UniConstants.COLOR_SENSOR_SLOT_2_STRING);
    }

    /**
     * Manually reset the encoder position.
     * Use for emergency re-zeroing if needed.
     */
    public void reset() {
        motor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ========== POSITION GETTERS ==========

    /**
     * Gets the current rotary position.
     *
     * @return Current position in radians
     */
    public double getPosition() {
        return currentPosition;
    }

    /**
     * Gets the target rotary position.
     *
     * @return Target position in radians
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    // ========== POSITION CHECKING ==========

    /**
     * Checks if rotary is within tolerance of target position.
     *
     * @return true if position error is less than POSITION_TOLERANCE
     */
    public Boolean withinRangeBool() {
        return Math.abs(MathUtil.piWraparound(currentPosition - targetPosition - offset)) < positionTolerance;
    }

    /**
     * Command that completes when rotary reaches target position.
     * Useful for waiting until chamber is aligned before launching.
     *
     * @return Command that finishes when position is reached
     */
    public Command withinRange() {
        return new LambdaCommand("Rotary Within Range?")
                .setIsDone(this::withinRangeBool);
    }

    // ========== LOCK/UNLOCK COMMANDS ==========

    /**
     * Locks the rotary by disabling feedforward.
     * Used to hold position firmly without allowing drift.
     */
    public Command lock = new InstantCommand(() -> {
        this.locked = true;
        fn = 0.0;  // Disable feedforward for stronger holding
        mCon.setPDFL(p, d, fn, l);
    });

    /**
     * Unlocks the rotary by enabling feedforward.
     * Allows smoother motion between positions.
     */
    public Command unlock = new InstantCommand(() -> {
        this.locked = false;
        fn = f;  // Re-enable feedforward for motion
        mCon.setPDFL(p, d, fn, l);
    });

    // ========== HOMING ROUTINE ==========

    /**
     * Phase 1 of homing: Slowly rotate backward until wall is detected.
     * Distance sensor reads < 0.3" when wall is found.
     */
    private Command findWall() {
        return new LambdaCommand("Homing Rotary: Finding Wall")
                .setStart(() -> {
                    findWall = true;
                    distSensorOutput = distSensor.getDistance(DistanceUnit.INCH);
                })
                .setIsDone(() -> distSensorOutput < 0.3)
                .setStop((interrupted) -> {
                    findWall = false;
                });
    }

    /**
     * Phase 2 of homing: Rotate forward slowly until edge is found.
     * Distance sensor reads > 0.5" when edge is detected.
     */
    private Command findEdge() {
        return new LambdaCommand("Homing Rotary: Finding Edge")
                .setStart(() -> {
                    findEdge = true;
                    distSensorOutput = distSensor.getDistance(DistanceUnit.INCH);
                })
                .setIsDone(() -> distSensorOutput > 0.5)
                .setStop((interrupted) -> {
                    findEdge = false;
                });
    }

    /**
     * Phase 3 of homing: Calculate and store the calibration offset.
     * Adds 0.16 radians to compensate for edge detection point.
     */
    private final Command finishHoming = new InstantCommand(() -> {
        offset = currentPosition + 0.16;
        homingDone = true;
    });

    /**
     * Resets homing state to allow re-homing if needed.
     */
    public final Command startRotary = new InstantCommand(() -> {
        homingDone = false;
    });

    /**
     * Complete homing sequence: find wall → find edge → calculate offset.
     * Run this at the start of autonomous to establish absolute position.
     */
    public SequentialGroup home = new SequentialGroup(
            findWall(),
            findEdge(),
            finishHoming
    );

    // ========== CHAMBER SELECTION ==========

    /**
     * Internal method to set target to a specific chamber.
     *
     * @param chamber Chamber number (1, 2, or 3)
     */
    private void Chamber(int chamber) {
        switch (chamber) {
            case 1:
                targetPosition = Chamber.ONE.angle;
                currentChamber = 1;
                break;
            case 2:
                targetPosition = Chamber.TWO.angle;
                currentChamber = 2;
                break;
            case 3:
                targetPosition = Chamber.THREE.angle;
                currentChamber = 3;
                break;
        }
        // Flag that colors should be updated when rotary stops
        shouldUpdateColors = true;
    }

    /**
     * Rotates to the previous chamber (3→2→1→3).
     * Useful for manual cycling through chambers.
     */
    public Command previousChamber = new InstantCommand(() -> {
        Chamber(currentChamber == 1 ? 3 : currentChamber - 1);
    });

    /**
     * Rotates to the next chamber (1→2→3→1).
     * Useful for manual cycling through chambers.
     */
    public Command nextChamber = new InstantCommand(() -> {
        Chamber(currentChamber == 3 ? 1 : currentChamber + 1);
    });

    /**
     * Rotates to whichever chamber contains a GREEN ball.
     * Searches all three chambers and selects the first match.
     */
    public Command greenChamber = new InstantCommand(() -> {
        for (int i = 0; i < CHAMBERS.length; i++) {
            if (CHAMBERS[i].ball == Ball.GREEN) {
                Chamber(i + 1);
                return;
            }
        }
    });

    /**
     * Rotates to whichever chamber contains a PURPLE ball.
     * Searches all three chambers and selects the first match.
     */
    public Command purpleChamber = new InstantCommand(() -> {
        for (int i = 0; i < CHAMBERS.length; i++) {
            if (CHAMBERS[i].ball == Ball.PURPLE) {
                Chamber(i + 1);
                return;
            }
        }
    });

    // ========== HALF CHAMBER MODE ==========

    /**
     * Enables half-chamber mode.
     * Positions rotary halfway between chambers for easier loading.
     */
    public Command setHalfChamberOn = new InstantCommand(() -> {
        this.halfChamber = true;
    });

    /**
     * Disables half-chamber mode.
     * Returns to normal chamber-aligned positioning.
     */
    public Command setHalfChamberOff = new InstantCommand(() -> {
        this.halfChamber = false;
    });

    /**
     * Toggles half-chamber mode on/off.
     * Convenient for driver control toggle button.
     */
    public Command toggleHalfChamber = new InstantCommand(() -> {
        this.halfChamber = !halfChamber;
    });

    // ========== COLOR DETECTION ==========

    /**
     * Checks if value 'a' is within tolerance of value 'b'.
     *
     * @param a   First value
     * @param b   Second value
     * @param tol Tolerance (±)
     * @return true if |a - b| <= tol
     */
    private boolean close(double a, double b, double tol) {
        return Math.abs(a - b) <= tol;
    }

    /**
     * Classifies a ball based on color sensor RGB values.
     * Compares against known GREEN and PURPLE color signatures.
     *
     * @param c   Color sensor to read from
     * @param tol Tolerance for color matching
     * @return Ball type (GREEN, PURPLE, or NULL)
     */
    private Ball classify(NormalizedColorSensor c, double tol) {
        // Read normalized RGB values (0-1 range)
        double r = c.getNormalizedColors().red;
        double g = c.getNormalizedColors().green;
        double b = c.getNormalizedColors().blue;

        // Check if color matches GREEN signature
        if (close(r, greenColor[0], higherGreenColors[0] - greenColor[0]) &&
                close(g, greenColor[1], higherGreenColors[1] - greenColor[1]) &&
                close(b, greenColor[2], higherGreenColors[2] - greenColor[2])) {
            return Ball.GREEN;
        }

        // Check if color matches PURPLE signature
        if (close(r, purpleColor[0],higherPurpleColors[0] - purpleColor[0]) &&
                close(g, purpleColor[1], higherPurpleColors[1] - purpleColor[1]) &&
                close(b, purpleColor[2], higherPurpleColors[2] - purpleColor[2])) {
            return Ball.PURPLE;
        }

        // No match found
        return Ball.NULL;
    }

    /**
     * Updates the ball type for all three chambers.
     * Reads from all color sensors and classifies each ball.
     * Only called when rotary is stopped and aligned.
     */
    private void updateChamberColor() {
        for (int i = 0; i < 3; i++) {
            CHAMBERS[i].ball = classify(colorSensors[i], colorTolerance);
        }
        shouldUpdateColors = false;
    }

    // ========== PERIODIC UPDATE ==========

    /**
     * Called every loop cycle by NextFTC's command scheduler.
     * Handles position control, homing, and color detection.
     */
    @Override
    public void periodic() {
        // Calculate half-chamber offset (60° = π/3 radians)
        halfOffset = halfChamber ? Math.PI / 3 : 0;
        mCon.setPDFL(p, d, f, l);

        // Read current position from encoder and wrap to [-π, π]
        currentPosition = MathUtil.piWraparound(
                (Encoder.getCurrentPosition() / ticksPerRotation) * 2 * Math.PI
        );

        // Update controller target (includes chamber angle + half offset + calibration)
        mCon.setTarget(MathUtil.piWraparound(targetPosition + halfOffset + offset));

        // Calculate control output
        mCon.update(currentPosition);
        double power = mCon.runPDFL(0.009);

        // Handle different operating modes
        if (findWall) {
            // Homing phase 1: Move backward to find wall
            motor.setPower(-0.5);
            distSensorOutput = distSensor.getDistance(DistanceUnit.INCH);

        } else if (findEdge) {
            // Homing phase 2: Move forward slowly to find edge
            motor.setPower(0.2);
            distSensorOutput = distSensor.getDistance(DistanceUnit.INCH);

        } else if (homingDone) {
            // Homing complete: hold position at zero power
            motor.setPower(0);

        } else {
            motor.setPower(power);
        }

        // Update chamber colors when stopped and aligned (not in half-chamber mode)
        if (shouldUpdateColors && power <= stoppedPowerThreshold && !halfChamber) {
            updateChamberColor();
        }
    }

    // ========== DEBUG INFORMATION ==========

    /**
     * Generates comprehensive debug string with all subsystem state.
     * Includes position, controller tuning, chamber states, and sensor data.
     *
     * @return Multi-section formatted debug string
     */

    public String debugText() {
        StringBuilder sb = new StringBuilder();

        sb.append("locked: ").append(locked);
        sb.append("\np: ").append(p);
        sb.append("\nd: ").append(d);
        sb.append("\nf: ").append(f);
        sb.append("\nl: ").append(l);
        sb.append("\nfn: ").append(fn);

        sb.append("\ncurrentChamber: ").append(currentChamber);
        sb.append("\ntargetPosition: ").append(targetPosition);
        sb.append("\ncurrentPosition: ").append(currentPosition);

        sb.append("\nhalfChamber: ").append(halfChamber);
        sb.append("\nchamberOffset: ").append(halfOffset);
        sb.append("\nOffset: ").append(offset);

        sb.append("\nticksPerRotation: ").append(ticksPerRotation);
        sb.append("\nshouldUpdateColors: ").append(shouldUpdateColors);

        // show each chamber's angle + ball state
        sb.append("\n\n=== Chamber States ===");
        for (int i = 0; i < 3; i++) {
            Chamber ch = CHAMBERS[i];
            sb.append("\nChamber ").append(i + 1)
                    .append(" | angle: ").append(ch.angle)
                    .append(" | ball: ").append(ch.ball);
        }

        // show raw RGB from each sensor
        sb.append("\n\n=== Sensor Colors ===");
        for (int i = 0; i < 3; i++) {
            NormalizedColorSensor c = colorSensors[i] != null ? colorSensors[i] : null;
            if (c == null) {
                sb.append("\nSensor ").append(i + 1).append(": NULL");
            } else {
                sb.append("\nSensor ").append(i + 1)
                        .append(" | R: ").append((c.getNormalizedColors().red))
                        .append(" G: ").append((c.getNormalizedColors().green))
                        .append(" B: ").append((c.getNormalizedColors().blue));
            }


            assert c != null;
            if((c.getNormalizedColors().red >= lowerGreenColors[0]) && (c.getNormalizedColors().red <= higherGreenColors[0])) {
                sb.append("\n Red in range for green");
            }   if((c.getNormalizedColors().blue >= lowerGreenColors[1]) && (c.getNormalizedColors().blue <= higherGreenColors[1])) {
                sb.append("\n Blue in range for green");
            }   if((c.getNormalizedColors().green >= lowerGreenColors[2]) && (c.getNormalizedColors().green <= higherGreenColors[2])) {
                sb.append("\n Green in range for green");
            }
            if((c.getNormalizedColors().red >= lowerPurpleColors[0]) && (c.getNormalizedColors().red <= higherPurpleColors[0])) {
                sb.append("\n Red in range for purple");
            }   if((c.getNormalizedColors().blue >= lowerPurpleColors[1]) && (c.getNormalizedColors().blue <= higherPurpleColors[1])) {
                sb.append("\n Blue in range for purple");
            }   if((c.getNormalizedColors().green >= lowerPurpleColors[2]) && (c.getNormalizedColors().green <= higherPurpleColors[2])) {
                sb.append("\n Green in range for purple");
            }
        }

        sb.append("\n\n=== Sensor Distance ===");
        sb.append("\nDist: ").append(distSensorOutput);

        // controller internals (if available)
        sb.append("\n\n=== Controller ===");
        sb.append("\ncontroller target: ").append(mCon.getTarget());

        return sb.toString();
    }
}


