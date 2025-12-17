package org.firstinspires.ftc.teamcode.subsystems.UpperLevel;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.MathUtil;
import org.firstinspires.ftc.teamcode.Util.Timer;

import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MidLevel.RotarySubsystem;

import dev.nextftc.core.subsystems.Subsystem;
@Configurable
public class LauncherSubsystem implements Subsystem {
    public OuttakeSubsystem outtakeSubsystem;
    public RotarySubsystem rotarySubsystem;
    private int currentState = 0;
   // public static double outtakeTarget = 1500;
    private int chamber = 0;
    private boolean start = false;
    private Timer timer = new Timer();
    private boolean reset = true;
    public static int servoTime = 1;

    private int shootCount = 0;

    public LauncherSubsystem(HardwareMap hMap) {
        outtakeSubsystem = new OuttakeSubsystem();
        rotarySubsystem = new RotarySubsystem(hMap, UniConstants.ROTARY_MOTOR_STRING);
    }

    public boolean getIsInPosition() {
//        if((rotarySubsystem.getPosition() > rotarySubsystem.getTargetPosition() - 0.1) && (rotarySubsystem.getPosition() < rotarySubsystem.getTargetPosition() + 0.1)) {
        if((Math.abs(MathUtil.piWraparound(rotarySubsystem.getPosition() - rotarySubsystem.getTargetPosition()))) > 0.06) {
           return true;
        }
        return false;
    }

    public boolean getStart() {return start;}
    public void setStart(boolean start) {this.start = start;}
    public void setReset(boolean reset) {this.reset = reset;}

    public void stateUpdate(int newState) {
        currentState = newState;
    }

    public void setShootCount(int count) {
        shootCount = count;
    }

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
    }
}
