package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.MathUtil;
import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.CommandGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.core.commands.Command;

@Configurable
public class Rotary implements Subsystem {
    // ========== SINGLETON PATTERN ==========

    /**
     * Singleton instance for easy access across OpModes
     */
    public static final Rotary INSTANCE = new Rotary();

    Timer timer = new Timer();

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
    public static double[] greenColor = {175.0, 0.81, 0.70};

    // The lower and upper sections of green colors
    public static double[] lowerGreenColors = {140.0, 0.55, 0.0009};
    public static double[] higherGreenColors = {180.0, 0.9, 0.04};

    /**
     * RGB values for PURPLE ball detection (normalized 0-1 scale)
     */

    // The lower and upper sections of purple colors
    public static double[] purpleColor = {130, 81, 70};
    public static double[] lowerPurpleColors = {200, 0.4, 0.002};
    public static double[] higherPurpleColors = {250, 0.6, 0.05};

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
    private boolean locked = true;

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

    public static double PICKUP_DELAY = 0.7;

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
    private boolean startOperation = false;

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
    private Rotary() {
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

    public Command startOpMode = new InstantCommand(() -> {
        startOperation = true;
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
                .setIsDone(() -> distSensorOutput < 0.3 && CHAMBERS[1].ball == Ball.NULL)
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
                .setIsDone(() -> distSensorOutput > 0.5 || CHAMBERS[1].ball != Ball.NULL)
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

    });

    /**
     * Complete homing sequence: find wall → find edge → calculate offset.
     * Run this at the start of autonomous to establish absolute position.
     */
    public SequentialGroup home = new SequentialGroup(
            new Delay(.5),
            findWall(),
            findEdge(),
            finishHoming
    );

    // ========== CHAMBER SWITCHING ==========


    /**
     * Rotates to the next chamber (1→2→3→1).
     * Useful for manual cycling through chambers.
     */
    public Command nextChamber = new InstantCommand(() -> {
        Chamber(currentChamber == 3 ? 1 : currentChamber + 1);
    });
    // Rotates the rotary to collect balls
    public Command rotateRotary = new SequentialGroup(
            new Delay(PICKUP_DELAY),
            nextChamber,
            new Delay(PICKUP_DELAY),
            nextChamber
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

    private double[] getColor(NormalizedColorSensor c) {
        NormalizedRGBA colors = c.getNormalizedColors();
        double r = colors.red;
        double g = colors.green;
        double b = colors.blue;



        double max = Math.max(r, Math.max(g, b));
        double min = Math.min(r, Math.min(g, b));
        double delta = max - min;

        double h = 0;
        if (delta != 0) {
            if      (max == r) h = 60 * (((g - b) / delta) % 6);
            else if (max == g) h = 60 * (((b - r) / delta) + 2);
            else               h = 60 * (((r - g) / delta) + 4);
            if (h < 0) h += 360;
        }

        return new double[] {
                h,                                  // Hue:        [0, 360)
                (max == 0) ? 0 : delta / max,       // Saturation: [0, 1]
                max                                 // Value:      [0, 1]
        };
    }

    /**
     * Classifies a ball based on color sensor RGB values.
     * Compares against known GREEN and PURPLE color signatures.
     *
     * @param c Color sensor to read from
     * @return Ball type (GREEN, PURPLE, or NULL)
     */
    private Ball classify(NormalizedColorSensor c) {
        double[] hsv = getColor(c);


        // Check if color matches GREEN signature
        if ((hsv[0] <= higherGreenColors[0] && hsv[0] >= lowerGreenColors[0]) &&
                (hsv[1] <= higherGreenColors[1] && hsv[1] >= lowerGreenColors[1]) &&
                (hsv[2] <= higherGreenColors[2] && hsv[2] >= lowerGreenColors[2])) {
            return Ball.GREEN;
        }

        // Check if color matches PURPLE signature
        if ((hsv[0] <= higherPurpleColors[0] && hsv[0] >= lowerPurpleColors[0]) &&
                (hsv[1] <= higherPurpleColors[1] && hsv[1] >= lowerPurpleColors[1]) &&
                (hsv[2] <= higherPurpleColors[2] && hsv[2] >= lowerPurpleColors[2])) {
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
            CHAMBERS[i].ball = classify(colorSensors[i]);
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
            updateChamberColor();

        } else if (findEdge) {
            // Homing phase 2: Move forward slowly to find edge
            motor.setPower(0.2);
            distSensorOutput = distSensor.getDistance(DistanceUnit.INCH);
            updateChamberColor();

        } else if (!startOperation) {
            // Homing complete: hold position at zero power
            motor.setPower(0);

        } else {
            motor.setPower(power);
        }

        // Update chamber colors when stopped and aligned (not in half-chamber mode)
        if (shouldUpdateColors && power <= stoppedPowerThreshold && !halfChamber) {
            updateChamberColor();
        }

        if ( timer.hasElapsedSeconds(.5) && !shouldUpdateColors) {
            timer.reset();
            shouldUpdateColors = true;
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


        return sb.toString();
    }
    public String debugColors() {
        StringBuilder sb = new StringBuilder();
        // show raw RGB from each sensor
        sb.append("\n\n=== Sensor Colors ===");
        for (int i = 0; i < 3; i++) {
            double[] hsv = getColor(colorSensors[i]);
            NormalizedColorSensor c = colorSensors[i] != null ? colorSensors[i] : null;
            if (c == null) {
                sb.append("\nSensor ").append(i + 1).append(": NULL");
            } else {

                sb.append("\nSensor ").append(i + 1)
                        .append(" | \nH: ").append(hsv[0])
                        .append(" \nS: ").append(hsv[1])
                        .append(" \nV: ").append(hsv[2]);
            }


            assert c != null;
            if((hsv[0] >= lowerGreenColors[0]) && (hsv[0] <= higherGreenColors[0])) {
                sb.append("\n Hue in range for green");
            }   else {
                sb.append("\n Hue not in range for green");
            }
            if((hsv[1] >= lowerGreenColors[1]) && (hsv[1] <= higherGreenColors[1])) {
                sb.append("\n Saturation in range for green");
            }  else {
                sb.append("\n Saturation not in range for green");
            }
            if((hsv[2] >= lowerGreenColors[2]) && (hsv[2] <= higherGreenColors[2])) {
                sb.append("\n Value in range for green");
            } else {
                sb.append("\n Value not in range for green");
            }

            if((hsv[0] >= lowerPurpleColors[0]) && (hsv[0] <= higherPurpleColors[0])) {
                sb.append("\n Hue in range for purple");
            }   else {
                sb.append("\n Hue not in range for purple");
            }
            if((hsv[1] >= lowerPurpleColors[1]) && (hsv[1] <= higherPurpleColors[1])) {
                sb.append("\n Saturation in range for purple");
            }  else {
                sb.append("\n Saturation not in range for purple");
            }
            if((hsv[2] >= lowerPurpleColors[2]) && (hsv[2] <= higherPurpleColors[2])) {
                sb.append("\n Value in range for purple");
            } else {
                sb.append("\n Value not in range for purple");
            }


        }

        sb.append("\n-- General Colors --");
        sb.append("\n Hue Upper Value Green: ");sb.append(higherGreenColors[0]);
        sb.append("\n Saturation Upper Value Green: ");sb.append(higherGreenColors[1]);
        sb.append("\n Value Upper Value Green: ");sb.append(higherGreenColors[2]);

        sb.append("\n Hue Lower Value Green: ");sb.append(lowerGreenColors[0]);
        sb.append("\n Saturation Lower Value Green: ");sb.append(lowerGreenColors[1]);
        sb.append("\n Value Lower Value Green: ");sb.append(lowerGreenColors[2]);

        sb.append("\n Hue Higher Value Purple: ");sb.append(higherPurpleColors[0]);
        sb.append("\n Saturation Higher Value Purple: ");sb.append(higherPurpleColors[1]);
        sb.append("\n Value Higher Value Purple: ");sb.append(higherPurpleColors[2]);

        sb.append("\n Hue Lower Value Purple: ");sb.append(lowerPurpleColors[0]);
        sb.append("\n Saturation Lower Value Purple: ");sb.append(lowerPurpleColors[1]);
        sb.append("\n Value Lower Value Purple: ");sb.append(lowerPurpleColors[2]);



        sb.append("\n\n=== Sensor Distance ===");
        sb.append("\nDist: ").append(distSensorOutput);

        // controller internals (if available)
        sb.append("\n\n=== Controller ===");
        sb.append("\ncontroller target: ").append(mCon.getTarget());

        return sb.toString();
    }
}


