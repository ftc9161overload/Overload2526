package org.firstinspires.ftc.teamcode.subsystems;
import dev.nextftc.core.subsystems.Subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.PDFLController;

@Configurable
public class IntakeSubystem implements Subsystem {
    private boolean isOn = false;
    private final DcMotorEx motor;
    private final double motorSpeed = 0.2;
    private PDFLController mCon = new PDFLController(0.1, 0,0,0);

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

    // Toggles the isOn bool
    public void toggle() {
        this.isOn = !isOn;
    }

    public void debug(double power) {
        motor.setPower(power);
    }

    // Runs the motor if isOn is True with the motorSpeed
    @Override
    public void periodic() {
        if (isOn){
            mCon.update(motor.getVelocity());
            motor.setPower(mCon.runPDFL(.1));
        }
    }
    public String debugText() {
        return "motorSpeed: " + motorSpeed + "\nPDFL: " + mCon.runPDFL(0.1) + "\nisOn: " + isOn;
    }

}
