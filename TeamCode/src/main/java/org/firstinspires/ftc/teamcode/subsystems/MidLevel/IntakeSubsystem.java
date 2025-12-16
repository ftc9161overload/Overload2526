package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import dev.nextftc.core.subsystems.Subsystem;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class IntakeSubsystem implements Subsystem {

    private final DcMotorEx motor;
    public double motorSpeed = 0.5;

    public IntakeSubsystem(String motorName, HardwareMap hMap) {
        motor = hMap.get(DcMotorEx.class, motorName);
    }

    public void run() {
        motor.setPower(motorSpeed);
    }

    public void stop() {
        motor.setPower(0);
    }
}