package org.firstinspires.ftc.teamcode.subsystems;
import dev.nextftc.core.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.PDFLController;

public class IntakeSubystem implements Subsystem {
    private boolean isOn = false;
    private final DcMotorEx motor;
    private final double motorSpeed = 0.2;
    private PDFLController mCon;

    public IntakeSubystem(String motor, HardwareMap hMap){
        this.motor = hMap.get(DcMotorEx.class, motor);
    }

    // Sets isOn to the new value (Setter)
    public void set(boolean isOn) {
        this.isOn = isOn;
    }

    // Returns the current state of isOn (Getter)
    public boolean get() {
        return isOn;
    }

    // Runs the motor if isOn is True with the motorSpeed
    @Override
    public void periodic() {
        if (isOn){
            mCon.update(motor.getVelocity());
            motor.setPower(mCon.runPDFL(.1));
        }
    }

}
