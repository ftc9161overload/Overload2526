package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import dev.nextftc.core.subsystems.Subsystem;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class IntakeSubsystem implements Subsystem {

    private final DcMotorEx motor;
    public double motorSpeed = 0.5;

    private boolean running = false;

    public IntakeSubsystem(String motorName, HardwareMap hMap) {
        motor = hMap.get(DcMotorEx.class, motorName);
    }

    public void run() {
        running = true;
        motor.setPower(motorSpeed);
    }

    public void stop() {
        running = false;
        motor.setPower(0);
    }

    public void toggle() {
        if (running) {
            stop();
        } else {
            run();
        }
    }

    public boolean isRunning() {
        return running;
    }
}
