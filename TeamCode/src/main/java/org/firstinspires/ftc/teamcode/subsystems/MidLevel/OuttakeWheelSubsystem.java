package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class OuttakeWheelSubsystem implements Subsystem {
    private final MotorEx motor = new MotorEx(UniConstants.OUTTAKE_MOTOR_STRING);
    
    public static final OuttakeWheelSubsystem INSTANCE = new OuttakeWheelSubsystem();
    private OuttakeWheelSubsystem() {}

    // Diffrent speeds for the wheel to hit.
    public int[] targetSpeeds = {1800, 2000, 2800};
    private int targetSpeed = 0;
    private final PDFLController pdfl = new PDFLController(0.002, 0.0001, 0.05, 0.1);
    
    public Command setSpeed1 = new InstantCommand(() -> {
        targetSpeed = targetSpeeds[0];
    });
    public Command setSpeed2 = new InstantCommand(() -> {
        targetSpeed = targetSpeeds[1];
    });
    public Command setSpeed3 = new InstantCommand(() -> {
        targetSpeed = targetSpeeds[2];
    });
    
    public void periodic() {
        double currentSpeed = motor.getVelocity();
        pdfl.setTarget(targetSpeed);
        pdfl.update(currentSpeed);
        double power = pdfl.runPDFL(50);

        // clamp power to valid motor range
        power = Math.max(0.0, Math.min(1.0, power));
        motor.setPower(power);
    }
}