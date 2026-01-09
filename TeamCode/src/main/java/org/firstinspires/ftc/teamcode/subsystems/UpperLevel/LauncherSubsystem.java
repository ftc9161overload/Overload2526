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
import dev.nextftc.core.commands.utility.LambdaCommand;
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




    // Sets the rotary to half if it's not already, and it can only do that if the flipper is down
    public Command setHalfOn = new InstantCommand(() -> {
        if(RotarySubsystem.INSTANCE.halfChamber) {
            OuttakeFlipperSubsystem.INSTANCE.setFullOff.schedule();
            RotarySubsystem.INSTANCE.setHalfChamberOff.schedule();
        }
    });

    public Command setHalfOff = new InstantCommand(() -> {
        if(!RotarySubsystem.INSTANCE.halfChamber) {
            OuttakeFlipperSubsystem.INSTANCE.setFullOff.schedule();
            RotarySubsystem.INSTANCE.setHalfChamberOn.schedule();
        }
    });

    // Sets both the outtake wheel and moves the rotary to half chamber
    public Command setup() {
        return new LambdaCommand(("Setup"))
                .setStart(() -> {
                        if (!(OuttakeWheelSubsystem.INSTANCE.targetSpeed > 0)) {
                            OuttakeWheelSubsystem.INSTANCE.setSpeed1.schedule();
                        }
                    })
                .setIsDone(OuttakeWheelSubsystem.INSTANCE::withinRange);
//                .setRequirements(this);
    }


    // Launches an artifact
    public Command Launch1(){
        return new SequentialGroup(
                setup(),
                RotarySubsystem.INSTANCE.lock,
                RotarySubsystem.INSTANCE.withinRange(),
                OuttakeFlipperSubsystem.INSTANCE.setFullOn,
                new Delay(0.7),
                OuttakeFlipperSubsystem.INSTANCE.setFullOff,
                new Delay(0.4),
                RotarySubsystem.INSTANCE.unlock,
                RotarySubsystem.INSTANCE.nextChamber
        );
    }
    public Command Launch3() {
        return new SequentialGroup(
                Launch1(),
                new Delay(0.5),
                Launch1(),
                new Delay(0.5),
                Launch1()
        );
    }


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
