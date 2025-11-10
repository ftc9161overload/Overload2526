package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

import dev.nextftc.core.subsystems.Subsystem;

public class VisionSubsystem implements Subsystem {
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private final Telemetry telemetry;
    private boolean debugMode = false;

    private Timer timer = new Timer();

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

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

    // Enable or disable debug mode
    public void setDebugMode(boolean debugMode) {
        this.debugMode = debugMode;
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

    public AprilTagDetection getFirstTagData() {
        ArrayList<AprilTagDetection> detections = getDetections();
        if (!detections.isEmpty()) {
            return detections.get(0);
        }
        return null;
    }

    public void findPosition() {
        AprilTagDetection tagData = getFirstTagData();
        if (tagData != null) {
            // Logic to determine position based on tag ID
            if (tagData.id == 20 || true){
                // determine position if blue tag is seen
                double x = tagData.ftcPose.x;
                double y = tagData.ftcPose.y;
                double yaw = tagData.ftcPose.yaw;
                double realYaw = Math.toDegrees(Math.atan2(x,y));
                
                double hypotenuse = Math.hypot(x, y);
                
                if (debugMode) {
                    telemetry.addData("Tag ID: ", tagData.id);
                    telemetry.addData("X: ", x);
                    telemetry.addData("Y: ", y);
                    telemetry.addData("Yaw: ", yaw);
                    telemetry.addData("Real Yaw: ", realYaw);
                    telemetry.addData("Hypotenuse: ", hypotenuse);
                    telemetry.update();
                }

            } else if (tagData.id == 24){
                // determine position if red tag is seen
            }
        }
    }

    /** Stops the vision portal */
    public void stop() {
        visionPortal.close();
    }

    @Override
    public void periodic() {
        if (timer.hasElapsedSeconds(1.0)) {
            findPosition();
            timer.reset();
        }
    }
}
