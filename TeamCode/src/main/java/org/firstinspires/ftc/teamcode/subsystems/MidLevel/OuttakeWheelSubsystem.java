package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Util.Lerp;
import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class OuttakeWheelSubsystem implements Subsystem {
    private final MotorEx motor = new MotorEx(UniConstants.OUTTAKE_MOTOR_STRING);
    
    public static final OuttakeWheelSubsystem INSTANCE = new OuttakeWheelSubsystem();

    private double power = 0;

    private double currentSpeed = 0;
    private OuttakeWheelSubsystem() {}

    public int targetSpeed = 0;

    public boolean withinRangeBool() {
        return (Math.abs(targetSpeed - currentSpeed) < 60);
    }
    public Command withinRange() {
        return new LambdaCommand(("flywheel within range?"))
                .setIsDone(() -> withinRangeBool());
    }

    // Different speeds for the wheel to hit.
    public int[] targetSpeeds = {1800, 2200, 2600};

    public Lerp lerp = new Lerp(0,0,0);
    private PDFLController pdfl = new PDFLController(0.01, 0, 0.0, 0.000);

    public Command setSpeed1 = new InstantCommand(() -> {
        targetSpeed = targetSpeeds[0];
    });
    public Command setSpeed2 = new InstantCommand(() -> {
        targetSpeed = targetSpeeds[1];
    });
    public Command setSpeed3 = new InstantCommand(() -> {
        targetSpeed = targetSpeeds[2];
    });

    public Command setSpeedHigher = new InstantCommand(() -> {
        targetSpeed += 200;
        targetSpeed = Math.min(Math.max(targetSpeed, 1600),2600);
    });

    public Command setSpeedLower = new InstantCommand(() -> {
        targetSpeed -= 200;
        targetSpeed = Math.min(Math.max(targetSpeed, 1600),2600);
    });

    public Command turnOff = new InstantCommand(() -> {
        targetSpeed = 0;
    });

    public void initialize() {
        //pdfl.setErrorPower(1.4);
    }
    
    public void periodic() {

        currentSpeed = motor.getVelocity();
        pdfl.setTarget(targetSpeed);
        pdfl.update(currentSpeed);
        power += lerp.constantLerp(power,targetSpeed*0.00039411,1);//Math.min(pdfl.runPDFL(20),.1);

        // clamp power to valid motor range
        if (Double.isNaN(power)) {
            power = 0;
        }
        power = Math.max(0.0, Math.min(1.0, power));
        motor.setPower(power+pdfl.runPDFL(10));

    }

    public String debugString() {
        String reutrnStr =
                "\nOuttake vel: " + currentSpeed +
                "\nOuttake target: " + targetSpeed +
//                "\nOuttake PDFL: " + pdfl.runPDFL(20) +
                "\nOuttake Power: " + power +
                "\n lerp out: " + lerp.constantLerp(power,targetSpeed*0.000394113007885,1) +
//                "\nPDFL: " + pdfl.debugString();
                "";


        return reutrnStr;
    }
}