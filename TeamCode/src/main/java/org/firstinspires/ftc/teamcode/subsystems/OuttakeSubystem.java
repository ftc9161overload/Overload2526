package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.PDFLController;

import dev.nextftc.core.subsystems.Subsystem;

@Configurable
public class OuttakeSubystem implements Subsystem {
    private boolean isOn = false;
    private final DcMotorEx motor;
    private double motorSpeed = 0.8;
    private double targetVel = 1;
    private PDFLController mCon = new PDFLController(0.1, 0,0,0);

    public OuttakeSubystem(String motor, HardwareMap hMap){
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
        motor.setPower (power);
    }

    public void setVel(double vel ) {
        targetVel = vel;
    }

    // Runs the motor if isOn is True with the motorSpeed
    @Override
    public void periodic() {
        if (isOn){
            mCon.setTarget(targetVel);
            mCon.update(motor.getVelocity());
            motor.setPower(motorSpeed += mCon.runPDFL(.1));

        }
        else {
            motor.setPower(0);
        }
    }
    public String debug() {
        return "motorSpeed: " + motorSpeed + "\nPDFL: " + mCon.runPDFL(0.1) + "\nisOn: " + isOn;
    }
}
