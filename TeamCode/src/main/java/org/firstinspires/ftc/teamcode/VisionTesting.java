package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp(name = "Vision Testing w/ Subsystem", group = "TeleOp")
public class VisionTesting extends OpMode {

    private VisionSubsystem visionSubsystem;

    @Override
    public void init() {
        // Initialize vision subsystem
        visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
        visionSubsystem.setDebugMode(true); // Enable debug data output
    }

    @Override
    public void loop() {
        // Call periodic to trigger findPosition() and telemetry updates
        visionSubsystem.periodic();
    }

    @Override
    public void stop() {
        // Stop camera feed when done
        visionSubsystem.stop();
    }
}
