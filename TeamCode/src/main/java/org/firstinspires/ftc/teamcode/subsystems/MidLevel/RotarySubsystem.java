package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Util.MathUtil;
import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.core.commands.Command;

@Configurable
public class RotarySubsystem implements Subsystem {
    private final MotorEx motor = new MotorEx(UniConstants.ROTARY_MOTOR_STRING).brakeMode();
    public static final RotarySubsystem INSTANCE = new RotarySubsystem();
    public boolean locked = true;
    private RotarySubsystem() {}

    private static double p = 0.85, d = 0, f = 0, l = 0.12;
    private double fn = f;
    private PDFLControllerRadial mCon = new PDFLControllerRadial(p, d, fn,l);
    private int currentChamber = 3;
    private double currentPosition = 0;
    private double targetPosition = 0;
    private final double ticksPerRotation = (537.7*170)/38;
    private double chamber1 = 2*Math.PI*1/3;
    private double chamber2 = 2*Math.PI*2/3;
    private double chamber3 = 0;

    public boolean halfChamber = false;
    private double chamberOffset = 0;

    private double globalOffset;
    public void initialize() {
        globalOffset = MathUtil.piWraparound(((motor.getCurrentPosition() % ticksPerRotation) / ticksPerRotation * 2 * Math.PI));
        //currentChamber = 0;
        //currentPosition = 0;
        //chamberOffset = 0;
    }

    public void resetOffset() {
        globalOffset = MathUtil.piWraparound(((motor.getCurrentPosition() % ticksPerRotation) / ticksPerRotation * 2 * Math.PI));
        currentChamber = 0;
        currentPosition = 0;
        chamberOffset = 0;
    }
    // Getter method for returning the isOn boolean
    public double getPosition() {
        return currentPosition;
    }
    public double getTargetPosition() {
        return targetPosition;
    }
    // Setter method for setting isOn to an input value

    public Command lock = new InstantCommand(()->{
        this.locked = true;
        fn = 0.2;
    });
    public Command unlock = new InstantCommand(()->{
        this.locked = false;
        fn = f;
    });
    private void Chamber(int chamber) {
        if (chamber == 1) {
            targetPosition = chamber1;
            currentChamber = 1;
        }
        else if(chamber == 2) {
            targetPosition = chamber2;
            currentChamber = 2;
        }
        else if(chamber == 3) {
            targetPosition = chamber3;
            currentChamber = 3;
        }

    }

    public Command previousChamber = new InstantCommand(() -> {
        if(currentChamber == 1) {
            Chamber(3);
        }
        else if (currentChamber == 2) {
            Chamber(1);
        }
        else if (currentChamber == 3) {
            Chamber(2);
        }
    });
    
    public Command nextChamber = new InstantCommand(() -> {
        if (currentChamber == 1) {
            Chamber(2);
        }
        else if (currentChamber == 2) {
            Chamber(3);
        }
        else if (currentChamber == 3) {
            Chamber(1);
        }
    });

    //public void setHalfChamber(boolean halfChamber) {this.halfChamber = halfChamber;}
    public Command setHalfChamberOn = new InstantCommand(() -> {
        this.halfChamber = true;
    });

    public Command setHalfChamberOff = new InstantCommand(() -> {
        this.halfChamber = false;
    });
    
    public Command toggleHalfChamber = new InstantCommand(() -> {
        this.halfChamber = !halfChamber;
    });

    // Runs the motor if isOn is true
    @Override
    public void periodic() {
        if (halfChamber) {
            chamberOffset = Math.PI / 3;
        } else {
            chamberOffset = 0;
        }

        currentPosition = ((motor.getCurrentPosition() % ticksPerRotation) / ticksPerRotation * 2 * Math.PI);

        if (targetPosition > currentPosition) {
            mCon.setTarget(MathUtil.piWraparound( targetPosition + chamberOffset + globalOffset));
        } else {
            mCon.setTarget(MathUtil.piWraparound( targetPosition + Math.PI * 2 + chamberOffset + globalOffset));
        }

        mCon.update(currentPosition);
        motor.setPower(mCon.runPDFL(0.01));

    }
    
    public String debugText() {
        mCon.setPDFL(p,d,fn,l);
        return "locked: " + locked +
                "\np: " + p +
                "\nd: " + d +
                "\nf: " + f +
                "\nl: " + l +
                "\nfn: " + fn +
                "\ncurrentChamber: " + currentChamber +
                "\ncurrentPosition: " + currentPosition +
                "\ntargetPosition: " + targetPosition +
                "\nhalfChamber: " + halfChamber +
                "\nchamberOffset: " + chamberOffset +
                "\nglobalOffset: " + globalOffset;}
}
