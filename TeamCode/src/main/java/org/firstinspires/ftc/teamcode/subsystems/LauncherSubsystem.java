package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.subsystems.Subsystem;
public class LauncherSubsystem implements Subsystem{
    public OuttakeSubsystem outtakeSubsystem;
    public RotarySubsystem rotarySubsystem;
    private int currentState = 0;

    public LauncherSubsystem(HardwareMap hMap) {
        outtakeSubsystem = new OuttakeSubsystem(UniConstants.OUTTAKE_MOTOR_STRING, UniConstants.OUTTAKE_SERVO_STRING, hMap);
        rotarySubsystem = new RotarySubsystem(hMap, UniConstants.ROTARY_MOTOR_STRING);
    }

    public void stateUpdate(int newState) {
        currentState = newState;
    }

    public void update() {
        switch(currentState) {
            case 0:


                break;

            case 1:
                
                break;
        }

        outtakeSubsystem.periodic();
        rotarySubsystem.periodic();
    }
}
