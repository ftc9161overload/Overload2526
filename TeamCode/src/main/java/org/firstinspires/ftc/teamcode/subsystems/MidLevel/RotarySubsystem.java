package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Util.MathUtil;
import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
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

    private NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[3];
    private final int[] GreenColor = {0, 107, 102};
    private final int[] PurpleColor = {71, 35, 126};
    private final int colorTolerance = 10; // + or - tolerance is accounted for
    private boolean shouldUpdateColors = true;
    // The following is an Enum for each chamber.
    public enum Ball {
        PURPLE,
        GREEN,
        NULL
    }

    public enum Chamber {
        ONE(0, Ball.NULL),
        TWO(2*Math.PI*1/3, Ball.NULL),
        THREE(2*Math.PI*2/3, Ball.NULL);

        public final double angle;
        public Ball ball;

        Chamber(double angle, Ball ball) {
            this.angle = angle;
            this.ball = ball;
        }
    }

    private final Chamber[] CHAMBERS = {
            Chamber.ONE,
            Chamber.TWO,
            Chamber.THREE
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
        colorSensors[0] = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "colorSensor1");
        colorSensors[1] = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "colorSensor2");
        colorSensors[2] = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "colorSensor3");
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
            targetPosition = Chamber.ONE.angle;
            currentChamber = 1;
        }
        else if(chamber == 2) {
            targetPosition = Chamber.TWO.angle;
            currentChamber = 2;
        }
        else if(chamber == 3) {
            targetPosition = Chamber.THREE.angle;
            currentChamber = 3;
        }
        shouldUpdateColors = true;
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

    public Command greenChamber = new InstantCommand(() -> {
        if (Chamber.ONE.ball == Ball.GREEN) {
            Chamber(1);
        }
        else if (Chamber.TWO.ball == Ball.GREEN) {
            Chamber(2);
        }
        else if (Chamber.THREE.ball == Ball.GREEN) {
            Chamber(3);
        }
    });

    public Command purpleChamber = new InstantCommand(() -> {
        if (Chamber.ONE.ball == Ball.PURPLE) {
            Chamber(1);
        }
        else if (Chamber.TWO.ball == Ball.PURPLE) {
            Chamber(2);
        }
        else if (Chamber.THREE.ball == Ball.PURPLE) {
            Chamber(3);
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

    // Is b within a plus or minus tol
    private boolean close(double a, double b, double tol) {
        return Math.abs(a - b) <= tol;
    }

    private Ball classify(NormalizedRGBA c, double tol) {
        int r = (int)(c.red * 255);
        int g = (int)(c.green * 255);
        int b = (int)(c.blue * 255);

        if (close(r, GreenColor[0], tol) &&
                close(g, GreenColor[1], tol) &&
                close(b, GreenColor[2], tol))
            return Ball.GREEN;

        if (close(r, PurpleColor[0], tol) &&
                close(g, PurpleColor[1], tol) &&
                close(b, PurpleColor[2], tol))
            return Ball.PURPLE;

        return Ball.NULL;
    }

    private void updateChamberColor() {
        for (int i = 0; i < 3; i++) {
            NormalizedRGBA c = colorSensors[i].getNormalizedColors();
            CHAMBERS[i].ball = classify(c, colorTolerance);
        }
        shouldUpdateColors = false;
    }

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
        double power = mCon.runPDFL(0.009);
        motor.setPower(power);

        if (shouldUpdateColors && power <= 0.04 && !halfChamber){
            updateChamberColor();
        }
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
