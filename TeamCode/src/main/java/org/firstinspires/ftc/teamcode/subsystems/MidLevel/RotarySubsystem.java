package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Util.MathUtil;
import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.core.commands.Command;

@Configurable
public class RotarySubsystem implements Subsystem {
    private final MotorEx motor = new MotorEx(UniConstants.ROTARY_MOTOR_STRING).brakeMode().zeroed();
    public static final RotarySubsystem INSTANCE = new RotarySubsystem();
    private final MotorEx Encoder = new MotorEx("RoEn");
    public boolean locked = true;
    private RotarySubsystem() {}


    private static double p = 0.85, d = 0.01, f = 0, l = 0.12;
    private double fn = f;
    private PDFLControllerRadial mCon = new PDFLControllerRadial(p, d, fn,l);
    private int currentChamber = 1;
    private double currentPosition = 0;
    private double targetPosition = 0;
    private final double ticksPerRotation = 8192 * 170.0/32.0;//(537.7*170)/38;
//    private double chamber1 = 2*Math.PI*1/3;
//    private double chamber2 = 2*Math.PI*2/3;
//    private double chamber3 = 0;

    private final double[] chamberAngles = {
            0,
            2*Math.PI * 1/3,
            2*Math.PI * 2/3
    };

    /**
    * KEY:
     * 0 - NOTHING  |
     * 1 - PURPLE  |
     * 2 - GREEN
    */
    private int[] colorInPos = {
            0, 0, 0
    };


    public boolean halfChamber = false;
    private double chamberOffset = 0;

    public void reset() {
        motor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void initialize() {
        mCon.setPDFL(p,d,fn,l);
        motor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getPosition() {
        return currentPosition;
    }
    public double getTargetPosition() {
        return targetPosition;
    }

    public Command withinRange() {
        return new LambdaCommand(("Rotary Within Range?")).setIsDone(() -> Math.abs(currentPosition-targetPosition) < 0.01);
    }
    public Boolean withinRangeBool() {
        return Math.abs(MathUtil.piWraparound(currentChamber-targetPosition)) < 0.02;
    }
    public Command lock = new InstantCommand(()->{
        this.locked = true;
        fn = 0.0;
        mCon.setPDFL(p,d,fn,l);
    });
    public Command unlock = new InstantCommand(()->{
        this.locked = false;
        fn = f;
        mCon.setPDFL(p,d,fn,l);
    });
    private void Chamber(int chamber) {
        if (chamber == 1) {
            targetPosition = chamberAngles[0];
            currentChamber = 1;
        }
        else if(chamber == 2) {
            targetPosition = chamberAngles[1];
            currentChamber = 2;
        }
        else if(chamber == 3) {
            targetPosition = chamberAngles[2];
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

    public Command setHalfChamberOn = new InstantCommand(() -> {
        this.halfChamber = true;
    });

    public Command setHalfChamberOff = new InstantCommand(() -> {
        this.halfChamber = false;
    });
    
    public Command toggleHalfChamber = new InstantCommand(() -> {
        this.halfChamber = !halfChamber;
    });

    @Override
    public void periodic() {
        if (halfChamber) {
            chamberOffset = Math.PI / 3;
        } else {
            chamberOffset = 0;
        }

        currentPosition = MathUtil.piWraparound((Encoder.getCurrentPosition() / ticksPerRotation) * 2*Math.PI);

        mCon.setTarget(MathUtil.piWraparound( targetPosition + chamberOffset));

        mCon.update(currentPosition);
        motor.setPower(mCon.runPDFL(0.009));

    }
    
    public String debugText() {
        //mCon.setPDFL(p,d,fn,l);
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
                "\nchamberOffset: " + chamberOffset
               ;}
}
