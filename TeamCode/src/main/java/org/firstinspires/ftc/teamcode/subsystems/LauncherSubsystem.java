package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Util.Timer;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.subsystems.Subsystem;
public class LauncherSubsystem implements Subsystem {
    public OuttakeSubsystem outtakeSubsystem;
    public RotarySubsystem rotarySubsystem;
    private int currentState = 0;
    private double outtakeTarget = 2000;
    private int chamber = 0;
    private boolean start = false;
    private Timer timer;

    public LauncherSubsystem(HardwareMap hMap) {
        outtakeSubsystem = new OuttakeSubsystem(UniConstants.OUTTAKE_MOTOR_STRING, UniConstants.OUTTAKE_SERVO_STRING, hMap);
        rotarySubsystem = new RotarySubsystem(hMap, UniConstants.ROTARY_MOTOR_STRING);
    }

    public void setStart(boolean start) {this.start = start;}

    public void stateUpdate(int newState) {
        currentState = newState;
    }

    public void update() {
        switch(currentState) {
            case 0:
                chamber = 0;

                if(start) {
                    currentState = 1;
                }
                break;
            case 1:
                rotarySubsystem.setHalfChamber(true);
                currentState = 2;
                break;
            case 2:
                outtakeSubsystem.setVel(outtakeTarget);
                if((outtakeSubsystem.getVel() > outtakeTarget - 50) && (outtakeSubsystem.getVel() < outtakeTarget + 50)) {
                    currentState = 3;
                }
                break;
            case 3:
                outtakeSubsystem.setServo(UniConstants.engagementLevel.FULL_ON);
                if((outtakeSubsystem.getServoPos() > OuttakeSubsystem.fullOn - .005) && (outtakeSubsystem.getServoPos() < OuttakeSubsystem.fullOn + .005)) {
                    currentState = 4;
                }
                break;

            case 4:
                outtakeSubsystem.setServo(UniConstants.engagementLevel.FULL_OFF);
                if(outtakeSubsystem.getServoPos() == OuttakeSubsystem.fullOff) {
                    currentState = 5;
                }
                break;
            case 5:
                if(chamber == 2) {
                    currentState = 0;
                    break;
                }
                rotarySubsystem.nextChamber();
                chamber += 1;
                timer.reset();
                timer.hasElapsedSeconds(1);
                currentState = 2;
                break;
        }


        outtakeSubsystem.periodic();
        rotarySubsystem.periodic();
    }
}
