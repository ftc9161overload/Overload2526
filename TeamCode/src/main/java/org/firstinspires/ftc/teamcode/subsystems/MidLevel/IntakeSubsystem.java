package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.hardware.powerable.Powerable;
import dev.nextftc.hardware.powerable.SetPower;
import dev.nextftc.core.subsystems.Subsystem;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
public class IntakeSubsystem implements Subsystem {
    public static DcMotorEx motor;

    public static final IntakeSubsystem INSTANCE = new IntakeSubsystem();
    private IntakeSubsystem() { }

    public double motorSpeed = 0.5;

    public Command run = new SetPower((Powerable) motor,motorSpeed);
    public Command stop = new SetPower((Powerable) motor, 0.0);

}
