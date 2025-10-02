package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

import dev.nextftc.core.subsystems.Subsystem; // <-- from NextFTC framework

public class VisionSubsystem implements Subsystem {
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    public VisionSubsystem(HardwareMap hardwareMap) {
        // Build AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // Build Vision Portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();
    }

    /** Returns the full list of current AprilTag detections. */
    public ArrayList<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }

    /** Returns the first detected tag ID, or -1 if none seen. */

    public int getFirstTagId() {
        ArrayList<AprilTagDetection> detections = getDetections();
        if (!detections.isEmpty()) {
            return detections.get(0).id;
        }
        return -1;
    }


    /** Stops the vision portal */
    public void stop() {
        visionPortal.close();
    }

    @Override
    public void periodic() {
        // NextFTC calls this every loop cycle
        // You could put auto-logging or state updates here if you want
    }
}
