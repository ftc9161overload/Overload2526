package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class OuttakeFlipper implements Subsystem {
    private final ServoEx servo = new ServoEx(UniConstants.OUTTAKE_SERVO_STRING);
    public static double fullOff = 0.3, off = 0.5, on = 0.82, fullOn = 0.85;

    public static final OuttakeFlipper INSTANCE = new OuttakeFlipper();
    private OuttakeFlipper() {}

    public Command setOn = new SetPosition(servo, on).requires(this);
    public Command setOff = new SetPosition(servo, off).requires(this);
    public Command setFullOn = new SetPosition(servo, fullOn).requires(this);
    public Command setFullOff = new SetPosition(servo, fullOff).requires(this);
    /**
     * Returns debug telemetry information for the outtake flipper subsystem
     *
     * @return String containing current servo position and preset values
     */
    public String debugText() {
        return String.format("Outtake Flipper | Position: %.3f | fullOff: %.2f | off: %.2f | on: %.2f | fullOn: %.2f",
                servo.getPosition(),
                fullOff,
                off,
                on,
                fullOn);
    }
}