package org.firstinspires.ftc.teamcode.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MidLevel.Vision;

@TeleOp(name = "Vision Testing w/ Subsystem", group = "TeleOp")
public class VisionTesting extends OpMode {

    private Vision vision;

    @Override
    public void init() {
        // Initialize vision subsystem
        vision = new Vision();
        vision.setDebugMode(true); // Enable debug data output
    }

    @Override
    public void loop() {
        // Call periodic to trigger findPosition() and telemetry updates
        vision.periodic();
    }

    @Override
    public void stop() {
        // Stop camera feed when done
        vision.stop();
    }
}
