package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.RotarySubsystem;

// Rotary OpMode to test if the rotary is good or not
@TeleOp(name = "Rotary", group = "TeleOp")
public class RotaryOpMode extends OpMode {
    RotarySubsystem rotary;

    @Override
    public void init() {
        rotary = new RotarySubsystem(hardwareMap, "RM");
    }

    @Override
    public void loop() {

        // Should turn the rotary on/off on the press of the a button on gamepad 1
        if(gamepad1.aWasPressed()) {
            rotary.nextChamber();
        }
        rotary.periodic();
    }
}
