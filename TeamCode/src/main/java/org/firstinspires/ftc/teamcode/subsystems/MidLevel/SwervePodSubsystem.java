package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.MathUtil;
import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.Util.Vector2D;

import dev.nextftc.core.units.Angle;
import dev.nextftc.core.units.Distance;

/**
 * Subsystem representing a single swerve drive pod.
 * Each pod consists of:
 * - A drive motor for forward/backward movement
 * - A continuous rotation servo for steering
 * - An analog input for position feedback
 *
 * The pod can intelligently choose to flip 180° and reverse the motor
 * if that results in a faster rotation to the target angle.
 */
@Configurable
public class SwervePodSubsystem {

    // ========================================================================
    // HARDWARE COMPONENTS
    // ========================================================================

    private final CRServo servo;           // Steering servo
    private final DcMotorEx motor;         // Drive motor
    private final AnalogInput sIn;         // Analog position sensor for servo angle


    // ========================================================================
    // POSITION & GEOMETRY
    // ========================================================================

    private final Distance x;                // X offset of pod from robot center
    private final Distance y;                // Y offset of pod from robot center
    private final Angle posOffset;           // Angular offset based on position (atan2(x,y))

    private Angle servoOffset = Angle.fromDeg(0);        // Calibration offset for servo zero position
    private Angle currentPos = Angle.fromRad(0);         // Current servo angle
    private Angle targetPos = Angle.fromRad(0);          // Desired servo angle (non-flipped)
    private Angle flippedTargetPos = Angle.fromRad(0);   // Alternative target (180° rotated)
    private Angle setTargetPos = Angle.fromRad(0);       // The actual target being used (may be flipped)


    // ========================================================================
    // MOTOR CONTROL
    // ========================================================================

    private int motorDirection = 1;        // 1 = normal, -1 = reversed (when flipped)
    public double movementScaler = 1;      // Global speed multiplier


    // ========================================================================
    // SERVO CONTROL (PID-like controller)
    // ========================================================================

    public static double p = 0.4;                // Proportional gain
    public static double d = 0.01;               // Derivative gain
    public static double f = 0;                  // Feedforward gain
    public static double l = 0.1;                // (Likely a limiting factor or low-pass filter)
    public static double errorMin = 0.1;         // Minimum error threshold for servo control

    private final PDFLControllerRadial sCon; // Servo controller
    private boolean reverseServo = false;  // Flag to reverse servo direction


    // ========================================================================
    // DRIVE MODES & STATES
    // ========================================================================

    private UniConstants.swerveDriveType driveMode = UniConstants.swerveDriveType.TURN_GO;
    private boolean crossMode = false;     // Cross mode: pods form an X for defense


    // ========================================================================
    // FLIP OPTIMIZATION
    // ========================================================================

    private final Timer flipTimer = new Timer();
    private double flipCooldownSeconds = 0.2; // Prevents rapid flip toggling


    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Creates a new swerve pod subsystem.
     *
     * @param x X offset of the pod from robot center
     * @param y Y offset of the pod from robot center
     * @param servo Hardware name of the continuous rotation servo
     * @param motor Hardware name of the drive motor
     * @param analogInput Hardware name of the analog position sensor
     * @param hMap The robot's hardware map
     */
    public SwervePodSubsystem(Distance x, Distance y, String servo, String motor,
                              String analogInput, HardwareMap hMap) {
        this.x = x;
        this.y = y;
        this.servo = hMap.get(CRServo.class, servo);
        this.motor = hMap.get(DcMotorEx.class, motor);
        this.sIn = hMap.get(AnalogInput.class, analogInput);

        // Calculate the angular offset based on pod position
        this.posOffset = Angle.fromRad(Math.atan2(this.x.inMm, this.y.inMm));

        // Initialize the servo controller with default PID values
        this.sCon = new PDFLControllerRadial(p, d, f, l);
    }


    // ========================================================================
    // CONFIGURATION METHODS
    // ========================================================================

    /**
     * Sets the servo calibration offset in degrees.
     * This accounts for mechanical misalignment of the servo.
     */
    public void setServoOffsetDeg(double offset) {
        this.servoOffset = Angle.fromDeg(offset);
    }

    /**
     * Sets the servo calibration offset in radians.
     */
    public void setServoOffsetRad(double offset) {
        this.servoOffset = Angle.fromRad(offset);
    }

    /**
     * Sets the servo calibration offset as an Angle object.
     */
    public void setServoOffset(Angle offset) {
        this.servoOffset = offset;
    }

    /**
     * Sets whether the servo direction should be reversed.
     */
    public void setServoReverse(boolean set) {
        this.reverseServo = set;
    }

    /**
     * Configures PID values optimized for MK2 servos.
     */
    public void setServoMKII() {
        p = 0.3;
        d = 0.01;
        f = 0;
        l = 0.1;
        sCon.setPDFL(p, d, f, l);
        errorMin = 0.05;
    }

    /**
     * Configures PID values optimized for MK1 servos.
     */
    public void setServoMKI() {
        p = 0.8;
        d = 0.01;
        f = 0;
        l = 0.1;
        sCon.setPDFL(p, d, f, l);
        errorMin = 0.05;
    }

    /**
     * Sets custom PDFL controller values.
     */
    public void setPDFL(double p, double d, double f, double l) {
        this.p = p;
        this.d = d;
        this.f = f;
        this.l = l;
        sCon.setPDFL(p, d, f, l);
    }

    /**
     * Sets the drive mode (DEADZONE or TURN_GO).
     */
    public void setServoMode(UniConstants.swerveDriveType mode) {
        this.driveMode = mode;
    }

    /**
     * Sets motor configuration.
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setMotorMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public void setMotorDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }


    // ========================================================================
    // CROSS MODE (DEFENSIVE POSITION)
    // ========================================================================

    /**
     * Enables cross mode - pods turn to form an X pattern for defense.
     */
    public void cross() {
        crossMode = true;
    }

    /**
     * Disables cross mode - returns to normal operation.
     */
    public void unCross() {
        crossMode = false;
    }


    // ========================================================================
    // CORE UPDATE METHODS
    // ========================================================================

    /**
     * Main update loop with translational and rotational vectors.
     * This calculates the resultant vector for this specific pod based on its position.
     *
     * @param translational The robot's translational velocity vector
     * @param rotational The robot's rotational velocity (magnitude = rotation speed)
     */
    public void update(Vector2D translational, Vector2D rotational) {
        Vector2D resultant = getResultantVector(translational, rotational);
        update(resultant);
    }

    /**
     * Update using separate x, y, and rotation values.
     */
    public void update(double x, double y, double rotation) {
        update(new Vector2D(x, y), new Vector2D(rotation, 0));
    }

    /**
     * Update with movement scaling applied.
     */
    public void update(double x, double y, double rotation, double movementScaler) {
        this.movementScaler = movementScaler;
        update(new Vector2D(x, y), new Vector2D(rotation, 0));
    }

    /**
     * Core update method that handles servo positioning and motor control.
     * This is where the main swerve logic happens.
     *
     * @param drive The desired velocity vector for this pod
     */
    public void update(Vector2D drive) {
        // Read current servo position from analog sensor
        currentPos = readServoPosition();

        // Calculate target angles (normal and flipped)
        if (crossMode) {
            // In cross mode, point each pod at its offset angle
            targetPos = Angle.fromRad(MathUtil.piWraparound(posOffset.inRad));
            flippedTargetPos = Angle.fromRad(MathUtil.piWraparound(posOffset.inRad + Math.PI));
        } else {
            // Normal mode: point in the direction of the drive vector
            targetPos = Angle.fromRad(MathUtil.piWraparound(drive.angle() + servoOffset.inRad));
            flippedTargetPos = Angle.fromRad(MathUtil.piWraparound(targetPos.inRad + Math.PI));
        }

        // Calculate angular differences for both possible targets
        Angle diffTargetPos = Angle.fromRad(Math.abs(MathUtil.piWraparound(targetPos.inRad - currentPos.inRad)));
        Angle diffFlippedTargetPos = Angle.fromRad(Math.abs(MathUtil.piWraparound(flippedTargetPos.inRad - currentPos.inRad)));

        // Execute drive mode specific logic
        switch (driveMode) {
            case DEADZONE:
                updateDeadzoneMode(drive, diffTargetPos, diffFlippedTargetPos);
                break;

            case TURN_GO:
                updateTurnGoMode(drive, diffTargetPos, diffFlippedTargetPos);
                break;
        }
    }

    /**
     * Simplified update for when only servo control is needed (no motor drive).
     * Used when you want to position the servo without driving.
     */
    public void update() {
        currentPos = readServoPosition();

        Angle diffTargetPos = Angle.fromRad(Math.abs(MathUtil.piWraparound(targetPos.inRad - currentPos.inRad)));
        Angle diffFlippedTargetPos = Angle.fromRad(Math.abs(MathUtil.piWraparound(flippedTargetPos.inRad - currentPos.inRad)));

        // Choose flip vs normal based on which is faster (only when motor isn't moving)
        if (motor.getVelocity() < UniConstants.servoMovementDeadzone) {
            if (diffFlippedTargetPos.inRad < diffTargetPos.inRad && flipTimer.hasElapsedSeconds(flipCooldownSeconds)) {
                setTargetPos = flippedTargetPos;
                motorDirection = -1;
                flipTimer.reset();
            } else if (diffFlippedTargetPos.inRad >= diffTargetPos.inRad && flipTimer.hasElapsedSeconds(flipCooldownSeconds)) {
                setTargetPos = targetPos;
                motorDirection = 1;
                flipTimer.reset();
            }
        }

        // Update servo controller
        sCon.setTarget(setTargetPos.inRad);
        sCon.update(currentPos.inRad);

        // Apply servo power (with optional reversal)
        servo.setPower(reverseServo ? sCon.runPDFL(errorMin) : -sCon.runPDFL(errorMin));
    }


    // ========================================================================
    // DRIVE MODE IMPLEMENTATIONS
    // ========================================================================

    /**
     * DEADZONE mode: Simple but less optimal.
     * - Only updates servo target when joystick is outside deadzone
     * - Always drives motor at requested speed
     * - Does not use flip optimization
     */
    private void updateDeadzoneMode(Vector2D drive, Angle diffTargetPos, Angle diffFlippedTargetPos) {
        // Only set new target if we're actually trying to move
        if (drive.magnitude() > UniConstants.deadzone * movementScaler) {
            sCon.setTarget(targetPos.inRad);
        }

        // Update servo controller and apply power
        sCon.update(currentPos.inRad);
        servo.setPower(reverseServo ? sCon.runPDFL(errorMin) : -sCon.runPDFL(errorMin));

        // Drive motor at requested speed
        motor.setPower(drive.magnitude() * movementScaler);
    }

    /**
     * TURN_GO mode: Optimized swerve behavior.
     * - Chooses to flip 180° if it's faster
     * - Only drives when servo is pointed in the right direction
     * - Uses flip cooldown to prevent oscillation
     */
    private void updateTurnGoMode(Vector2D drive, Angle diffTargetPos, Angle diffFlippedTargetPos) {
        // Initialize target if it's the first run (default Angle is 0, not NaN)
        // Check if setTargetPos is still at its default initialized value
        if (setTargetPos.inRad == 0 && currentPos.inRad != 0) {
            setTargetPos = currentPos;
        }

        // Only update target direction when we're trying to move
        if (drive.magnitude() > UniConstants.deadzone * movementScaler) {
            // Only change direction when pod isn't already moving (prevents mid-motion flips)
            if (motor.getVelocity() < UniConstants.servoMovementDeadzone) {

                // Choose flip vs normal path based on which requires less rotation
                if (diffFlippedTargetPos.inRad < diffTargetPos.inRad && flipTimer.hasElapsedSeconds(flipCooldownSeconds)) {
                    // Flipping is faster - rotate 180° and reverse motor
                    setTargetPos = flippedTargetPos;
                    motorDirection = -1;
                    flipTimer.reset();

                } else if (diffFlippedTargetPos.inRad >= diffTargetPos.inRad && flipTimer.hasElapsedSeconds(flipCooldownSeconds)) {
                    // Normal path is faster
                    setTargetPos = targetPos;
                    motorDirection = 1;
                    flipTimer.reset();
                }
            }

            sCon.setTarget(setTargetPos.inRad);
        }

        // Update servo controller and apply power
        sCon.update(currentPos.inRad);
        servo.setPower(reverseServo ? sCon.runPDFL(errorMin) : -sCon.runPDFL(errorMin));

        // Calculate error from the FINAL chosen target (important!)
        Angle diffFinal = Angle.fromRad(Math.abs(MathUtil.piWraparound(setTargetPos.inRad - currentPos.inRad)));

        // Only drive when servo is pointed correctly
        if (diffFinal.inRad <= UniConstants.radialDeadzone) {
            motor.setPower(motorDirection * drive.magnitude() * movementScaler);
        } else {
            motor.setPower(0);  // Wait for servo to align
        }

        // Override motor in cross mode (defensive stance)
        if (crossMode) {
            motor.setPower(0);
        }
    }


    // ========================================================================
    // UTILITY METHODS
    // ========================================================================

    /**
     * Calculates the resultant velocity vector for this pod.
     * Combines translational movement with rotational movement based on pod position.
     *
     * @param translational Robot's translational velocity
     * @param rotational Robot's rotational velocity
     * @return The combined velocity vector for this specific pod
     */
    public Vector2D getResultantVector(Vector2D translational, Vector2D rotational) {
        // Rotate the rotational component by (pod offset + 90°) to get tangential velocity
        return translational.add(rotational.rotate(posOffset.inRad + Math.PI / 2));
    }

    /**
     * Reads the current servo angle from the analog sensor.
     *
     * @return Servo angle as an Angle object
     */
    private Angle readServoPosition() {
        return Angle.fromRad((sIn.getVoltage() / 3.3 * 2 * Math.PI) - Math.PI);
    }

    /**
     * Gets the angular offset of this pod from the robot center.
     */
    public Angle getRotationOffset() {
        return posOffset;
    }

    /**
     * Gets the current analog sensor position reading.
     */
    public Angle getAnalogInPos() {
        return readServoPosition();
    }


    // ========================================================================
    // MANUAL CONTROL / TESTING
    // ========================================================================

    /**
     * Manually sets the target servo position (for testing).
     * @param pos Angle to set as target
     */
    public void setPos(Angle pos) {
        setTargetPos = pos;
    }

    /**
     * Manually sets the target servo position in radians (for testing).
     * @param pos Position in radians
     */
    public void setPosRad(double pos) {
        setTargetPos = Angle.fromRad(pos);
    }

    /**
     * Manually sets servo power (bypasses controller, for testing).
     */
    public void setServoPower(double power) {
        servo.setPower(power);
    }


    // ========================================================================
    // DEBUG / TELEMETRY
    // ========================================================================

    /**
     * Returns debug information about the pod state.
     */
    public String debugText() {
        return "Servo Voltage: " + sIn.getVoltage() +
                "\nCurrent Pos: " + currentPos.inDeg + "° (" + currentPos.inDeg + " rad)" +
                "\nTarget Pos: " + targetPos.inDeg + "° (" + targetPos.inDeg + " rad)" +
                "\nSet Target Pos: " + setTargetPos.inDeg + "° (" + setTargetPos.inDeg + " rad)" +
                "\nPDFL Output: " + sCon.runPDFL(0.05) +
                "\nServo Offset: " + servoOffset.inDeg + "° (" + servoOffset.inDeg + " rad)" +
                "\nMotor Direction: " + motorDirection +
                "\n" + sCon.debugText();
    }
}