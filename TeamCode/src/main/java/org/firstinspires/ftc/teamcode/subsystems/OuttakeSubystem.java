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
    private double motorPower = 0.8;
    private double targetVel = 1;
    public static double p = 0.000001, d = 0, f = 0, l = 0;
    private PDFLController mCon = new PDFLController(p, d,f,l);

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
        motor.setPower(power);
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
            motor.setPower(motorPower += mCon.runPDFL(.1));

        }
        else {
            motor.setPower(0);
        }


        motorPower = Math.max(-1, Math.min(1,motorPower));
    }
    public String debugText() {
        mCon.setPDFL(p,d,f,l);
        return "motorSpeed: " + motorPower + "\nPDFL: " + mCon.runPDFL(0.1) + "\nisOn: " + isOn + "\nMotor Val: " + motor.getVelocity() + "\nTarget Vel: " + targetVel;
    }
}
