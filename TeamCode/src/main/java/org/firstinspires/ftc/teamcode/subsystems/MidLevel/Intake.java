package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;
import dev.nextftc.core.subsystems.Subsystem;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

@Configurable
public class Intake implements Subsystem {
    private final MotorEx motor = new MotorEx(UniConstants.INTAKE_MOTOR_STRING);

    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    public double motorSpeed = 0.5;

    public Command run = new SetPower(motor,motorSpeed).requires(this);
    public Command stop = new SetPower(motor, 0.0).requires(this);

}
