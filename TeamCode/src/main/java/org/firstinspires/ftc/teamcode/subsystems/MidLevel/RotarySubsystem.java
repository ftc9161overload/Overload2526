package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.MathUtil;
import org.firstinspires.ftc.teamcode.Util.PDFLControllerRadial;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.groups.SequentialGroup;
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

    private boolean findWall = false, findEdge = false;

    private final double ticksPerRotation = 8192 * 170.0/32.0;//(537.7*170)/38;

    private NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[3];
    private ColorRangeSensor distSensor;
    private double distSensorOutput;
    //private ColorRangeSensor thingy;

    private final double[] GreenColor = {0.009, 0.04, 0.028};
    private final double[] PurpleColor = {0.007, 0.009, 0.014};
    private final double colorTolerance = 0.001; // + or - tolerance is accounted for
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
    private double halfOffset = 0;
    private double offset = 0;

    public void reset() {
        motor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void initialize() {
        mCon.setPDFL(p,d,fn,l);
        motor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        colorSensors[0] = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "cs1");
        colorSensors[1] = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "cs2");
        colorSensors[2] = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "cs3");
        distSensor = ActiveOpMode.hardwareMap().get(ColorRangeSensor.class, "cs2");
    }

    public double getPosition() {
        return currentPosition;
    }
    public double getTargetPosition() {
        return targetPosition;
    }

    public Command withinRange() {
        return new LambdaCommand(("Rotary Within Range?")).setIsDone(() -> withinRangeBool());
    }
    public Boolean withinRangeBool() {
        return Math.abs(MathUtil.piWraparound(currentPosition-targetPosition-offset)) < 0.02;
    }
    public Command lock = new InstantCommand(()->{
        this.locked = true;
        fn = 0.0;
        mCon.setPDFL(p,d,fn,l);
    });



    private Command findWall() {
        return new LambdaCommand(("Homing Rotary: Finding Wall"))
                .setStart(() ->  {
                    findWall = true;
                    distSensorOutput = distSensor.getDistance(DistanceUnit.INCH);
                })
                .setIsDone(() ->
                        distSensorOutput < 0.3)
                .setStop((interrupted) ->{
                    findWall = false;
                });
    }
    private Command findEdge() {
        return new LambdaCommand(("Homing Rotary: Finding edge"))
                .setStart(() -> {
                    findEdge = true;
                    distSensorOutput = distSensor.getDistance(DistanceUnit.INCH);
                })
                .setIsDone(() -> distSensorOutput > 0.5)
                .setStop((interrupted) ->{
                    findEdge = false;
                });
    }

    private Command finishHoming = new InstantCommand(() -> {
        offset = currentPosition + 0.16;
    });

    public SequentialGroup home = new SequentialGroup(
            findWall(),
            findEdge(),
            finishHoming

    );


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

    private Ball classify(NormalizedColorSensor c, double tol) {
        double r = (c.getNormalizedColors().red * 255);
        double g = (c.getNormalizedColors().green * 255);
        double b = (c.getNormalizedColors().blue * 255);

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
            NormalizedColorSensor c = colorSensors[i];
            CHAMBERS[i].ball = classify(c, colorTolerance);
        }
        shouldUpdateColors = false;
    }

    @Override
    public void periodic() {
        if (halfChamber) {
            halfOffset = Math.PI / 3;
        } else {
            halfOffset = 0;
        }

        currentPosition = MathUtil.piWraparound((Encoder.getCurrentPosition() / ticksPerRotation) * 2*Math.PI);

        mCon.setTarget(MathUtil.piWraparound( targetPosition + halfOffset + offset));

        mCon.update(currentPosition);
        double power = mCon.runPDFL(0.009);


        if (findWall) {
            motor.setPower(-0.5);
            distSensorOutput = distSensor.getDistance(DistanceUnit.INCH);
        } else if (findEdge) {
            motor.setPower(0.2);
            distSensorOutput = distSensor.getDistance(DistanceUnit.INCH);

        } else {
            motor.setPower(power);
        }

        if (shouldUpdateColors && power <= 0.04 && !halfChamber){
            updateChamberColor();
        }
    }

    public String debugText() {
        StringBuilder sb = new StringBuilder();

        sb.append("locked: ").append(locked);
        sb.append("\np: ").append(p);
        sb.append("\nd: ").append(d);
        sb.append("\nf: ").append(f);
        sb.append("\nl: ").append(l);
        sb.append("\nfn: ").append(fn);

        sb.append("\ncurrentChamber: ").append(currentChamber);
        sb.append("\ntargetPosition: ").append(targetPosition);
        sb.append("\ncurrentPosition: ").append(currentPosition);

        sb.append("\nhalfChamber: ").append(halfChamber);
        sb.append("\nchamberOffset: ").append(halfOffset);
        sb.append("\nOffset: ").append(offset);

        sb.append("\nticksPerRotation: ").append(ticksPerRotation);
        sb.append("\nshouldUpdateColors: ").append(shouldUpdateColors);

        // show each chamber's angle + ball state
        sb.append("\n\n=== Chamber States ===");
        for (int i = 0; i < 3; i++) {
            Chamber ch = CHAMBERS[i];
            sb.append("\nChamber ").append(i+1)
                    .append(" | angle: ").append(ch.angle)
                    .append(" | ball: ").append(ch.ball);
        }

        // show raw RGB from each sensor
        sb.append("\n\n=== Sensor Colors ===");
        for (int i = 0; i < 3; i++) {
            NormalizedColorSensor c = colorSensors[i] != null ? colorSensors[i] : null;
            if (c == null) {
                sb.append("\nSensor ").append(i+1).append(": NULL");
            } else {
                sb.append("\nSensor ").append(i+1)
                        .append(" | R: ").append((c.getNormalizedColors().red))
                        .append(" G: ").append((c.getNormalizedColors().green))
                        .append(" B: ").append((c.getNormalizedColors().blue));
            }
        }

        sb.append("\n\n=== Sensor Distance ===");
        sb.append("\nDist: ").append(distSensorOutput);

        // controller internals (if available)
        sb.append("\n\n=== Controller ===");
        sb.append("\ncontroller target: ").append(mCon.getTarget());

        return sb.toString();
    }

}
