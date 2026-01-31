package org.firstinspires.ftc.teamcode.subsystems.MidLevel;

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
    // Vision processing components
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private final Telemetry telemetry;

    // Configuration flags
    private boolean debugMode = false;

    // Timer to control update frequency (prevents excessive processing)
    private final Timer timer = new Timer();

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Configure AprilTag processor with visual debugging features
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)           // Display tag ID on camera stream
                .setDrawAxes(true)            // Display coordinate axes on detected tags
                .setDrawCubeProjection(true)  // Display 3D cube overlay on tags
                .build();

        // Configure camera portal with webcam and processor
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))  // 640x480 resolution
                .enableLiveView(true)                      // Enable camera preview
                .build();
    }

    /**
     * Enables or disables telemetry debug output
     * @param debugMode true to enable debug telemetry
     */
    public void setDebugMode(boolean debugMode) {
        this.debugMode = debugMode;
    }

    /**
     * Gets all currently detected AprilTags
     * @return ArrayList of all detected AprilTags
     */
    public ArrayList<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }

    /**
     * Gets the ID of the first detected AprilTag
     * @return tag ID if detected, -1 if no tags found
     */
    public int getFirstTagId() {
        ArrayList<AprilTagDetection> detections = getDetections();
        return detections.isEmpty() ? -1 : detections.get(0).id;
    }

    /**
     * Gets the full detection data for the first detected AprilTag
     * @return AprilTagDetection object if tag found, null otherwise
     */
    public AprilTagDetection getFirstTagData() {
        ArrayList<AprilTagDetection> detections = getDetections();
        return detections.isEmpty() ? null : detections.get(0);
    }

    /**
     * Processes AprilTag detection to determine robot position
     * Currently configured for tag ID 20 (blue alliance)
     */
    private void findPosition() {
        AprilTagDetection tagData = getFirstTagData();

        // Exit early if no tag detected
        if (tagData == null) return;

        // Process blue alliance tag (ID 20)
        // Note: "|| true" forces this block to always execute - remove in production
        if (tagData.id == 20 || true) {
            // Extract position data from tag
            double x = tagData.ftcPose.x;      // Lateral distance (inches)
            double y = tagData.ftcPose.y;      // Forward distance (inches)
            double yaw = tagData.ftcPose.yaw;  // Tag's yaw angle (degrees)

            // Calculate actual bearing angle to tag
            double realYaw = Math.toDegrees(Math.atan2(x, y));

            // Calculate straight-line distance to tag
            double hypotenuse = Math.hypot(x, y);

            // Output debug information if enabled
            if (debugMode) {
                telemetry.addData("Tag ID", tagData.id);
                telemetry.addData("X", "%.2f in", x);
                telemetry.addData("Y", "%.2f in", y);
                telemetry.addData("Yaw", "%.2f°", yaw);
                telemetry.addData("Real Yaw", "%.2f°", realYaw);
                telemetry.addData("Distance", "%.2f in", hypotenuse);
                telemetry.update();
            }
        }
        // Placeholder for red alliance tag processing (ID 24)
        else if (tagData.id == 24) {
            // TODO: Implement red alliance position logic
        }
    }

    /**
     * Closes the vision portal and releases camera resources
     */
    public void stop() {
        visionPortal.close();
    }

    /**
     * Called periodically by the command scheduler
     * Updates position calculations at 1Hz to reduce CPU load
     */
    @Override
    public void periodic() {
        // Only process vision data once per second
        if (timer.hasElapsedSeconds(1.0)) {
            findPosition();
            timer.reset();
        }
    }
}
