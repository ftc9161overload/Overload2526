package org.firstinspires.ftc.teamcode.subsystems.UpperLevel;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.MathUtil;
import org.firstinspires.ftc.teamcode.Util.Timer;

import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeFlipperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.RotarySubsystem;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class LauncherSubsystem extends SubsystemGroup {
    public static final LauncherSubsystem INSTANCE = new LauncherSubsystem();
    private LauncherSubsystem() {
        super(
            OuttakeFlipperSubsystem.INSTANCE,
            OuttakeWheelSubsystem.INSTANCE,
            RotarySubsystem.INSTANCE
        );
    }

    public static int servoTime = 1;

    public boolean getIsInPosition() {
//        if((rotarySubsystem.getPosition() > rotarySubsystem.getTargetPosition() - 0.1) && (rotarySubsystem.getPosition() < rotarySubsystem.getTargetPosition() + 0.1)) {
        return (Math.abs(MathUtil.piWraparound(RotarySubsystem.INSTANCE.getPosition() - RotarySubsystem.INSTANCE.getTargetPosition()))) > 0.06;
    }


    // Sets the rotary to half if it's not already, and it can only do that if the flipper is down
    private Command setHalf = new InstantCommand(() -> {
        if(RotarySubsystem.INSTANCE.getHalfChamber()) {
            OuttakeFlipperSubsystem.INSTANCE.setOff.schedule();
            RotarySubsystem.INSTANCE.setHalfChamberOff.requires(OuttakeFlipperSubsystem.INSTANCE);
        }
    });

    // Sets both the outtake wheel and moves the rotary to half chamber
    private Command setup = new ParallelGroup(
        OuttakeWheelSubsystem.INSTANCE.setSpeed3,
            setHalf
    );

    // Turns the flipper servo on if the rotary is in an exact enough position
    private Command servoOn = new InstantCommand(() -> {
        if(getIsInPosition()) {
            OuttakeFlipperSubsystem.INSTANCE.setOn.requires();
        }
    });

    // Launches an artifact
    public Command Launch1 = new SequentialGroup(

            setup,
            new Delay(servoTime),
            servoOn,
            new Delay(servoTime),
            OuttakeFlipperSubsystem.INSTANCE.setOff.requires(),
            new Delay(servoTime),
            RotarySubsystem.INSTANCE.nextChamber.requires(OuttakeFlipperSubsystem.INSTANCE)

    ).named("Launch");

    public Command Launch2 = new SequentialGroup(
            Launch1,
            Launch1
    ).named("Launch2");

    public Command Launch3 = new SequentialGroup(
            Launch1,
            Launch1,
            Launch1,
            RotarySubsystem.INSTANCE.setHalfChamberOn
    ).named("Launch3");


    // Previous code
    // Sets the Outtake Wheel Subsystem to launch at the speed the method is named
    /*
    public void update() {
        switch(currentState) {
            case 0:
                start = false;
                chamber = 0;
                reset = false;


                break;
            case 1:
                timer.reset();
                stateUpdate(11);
                break;
            case 11:
                if (shootCount > 0) {
//                    stateUpdate(10);

                    outtakeSubsystem.setServo(UniConstants.engagementLevel.FULL_OFF);
                    if (timer.hasElapsedSeconds(1.2)) {
                        rotarySubsystem.nextChamber();
                        rotarySubsystem.setHalfChamber(false);
                        outtakeSubsystem.set(true);
                        stateUpdate(10);
                    }
                } else {
                    stateUpdate(0);
                }
                break;
            case 10:


                if(!reset) {
                    timer.reset();
                    reset = true;
                }
                rotarySubsystem.setHalfChamber(false);
//                if (rotarySubsystem.getHalfChamber()) {
//                    rotarySubsystem.setHalfChamber(false);
//                    if((getIsInPosition()) && (timer.hasElapsedSeconds(servoTime + 5))) {reset = false; stateUpdate(2);}
//                }
                if((getIsInPosition())) {reset = false; stateUpdate(2);}
                start = true;

                break;
            case 2:
                //outtakeSubsystem.setVel(outtakeTarget);
                outtakeSubsystem.set(true);
                if(!reset) {
                    timer.reset();
                    reset = true;
                }
                if((outtakeSubsystem.getVel() > outtakeSubsystem.getTargetVel() - 50) && (outtakeSubsystem.getVel() < outtakeSubsystem.getTargetVel() + 50)) {
                    if (timer.hasElapsedSeconds(0.5)) {
                        stateUpdate(3);
                        reset = false;
                    }
                }
                break;
            case 3:
                outtakeSubsystem.setServo(UniConstants.engagementLevel.ON);
                if(!reset) {
                    timer.reset();
                    reset = true;
                }
                if(timer.hasElapsedSeconds(servoTime)) {stateUpdate(4); reset = false;}

                break;

            case 4:
                outtakeSubsystem.setServo(UniConstants.engagementLevel.FULL_OFF);
                stateUpdate(5);
                break;
            case 5:
                if (shootCount > 0) {
                    stateUpdate(1);
                } else {
                    stateUpdate(0);
                }
                shootCount --;
                break;

        }

        debugText();
        outtakeSubsystem.periodic();
        rotarySubsystem.periodic();
    }
    public String debugText() {
        return "\nCurrent State: " + currentState + "\nlauncher chamber: " + chamber + "\ntime: " + timer.getTimeSeconds();
    }*/
}
