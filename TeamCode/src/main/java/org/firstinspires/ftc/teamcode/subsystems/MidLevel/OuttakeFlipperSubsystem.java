package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class OuttakeFlipperSubsystem implements Subsystem {
    private final ServoEx servo = new ServoEx(UniConstants.OUTTAKE_SERVO_STRING);
    public static double fullOff = 0.3, off = 0.5, on = 0.82, fullOn = 0.85;

    public static final OuttakeFlipperSubsystem INSTANCE = new OuttakeFlipperSubsystem();
    private OuttakeFlipperSubsystem() {}

    public Command setOn = new SetPosition(servo, on).requires(this);
    public Command setOff = new SetPosition(servo, off).requires(this);
    public Command setFullOn = new SetPosition(servo, fullOn).requires(this);
    public Command setFullOff = new SetPosition(servo, fullOff).requires(this);
}