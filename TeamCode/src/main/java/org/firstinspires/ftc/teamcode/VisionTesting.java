package org.firstinspires.ftc.teamcode;

import android.util.Size;


import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;

@TeleOp(name="Vision Testing", group="TeleOp")
public class VisionTesting extends OpMode{
    AprilTagProcessor myAprilTagProcessor = new AprilTagProcessor.Builder()
            .setDrawTagID(true)
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .build();

    ArrayList<AprilTagDetection> detections;  // list of all detections
    int code;

    @Override
    public void init() {

        VisionPortal myVisionPortal;
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();
        //TODO: Create color processor for balls

    }




    @Override
    public void init_loop() {

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {

        // ID code of current detection, in for() loop

        // Get a list of AprilTag detections.
        detections = myAprilTagProcessor.getDetections();

        // Cycle through through the list and process each AprilTag.
        for (AprilTagDetection detected : detections) {

            if (detected.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                code = detected.id;

                // Now take action based on this tag's ID code, or store info for later action.
                telemetry.addData("ID CODE: ", code);
                /*
                telemetry.addLine("ID CODE INDEX: " + detections.indexOf(detected) + " LOOKUP: ");
                telemetry.addData("metadata: ",detected.metadata.toString());
                telemetry.addData("center",detected.center);
                telemetry.addData("ftcPose", detected.ftcPose.toString());
                telemetry.addData("corners", detected.corners.toString());
                telemetry.addData("rawPose", detected.rawPose.toString());
                telemetry.addData("decisionMargin", detected.decisionMargin);
                telemetry.addData("frameAcquisitionNanoTime",detected.frameAcquisitionNanoTime);
                telemetry.addData("hamming", detected.hamming);
                telemetry.addData("robotPose",detected.robotPose);
                 */
                telemetry.addData("X: ", String.valueOf(detected.ftcPose.x));
                telemetry.addData("Y: ", String.valueOf(detected.ftcPose.y));
                telemetry.addData("Z: ", String.valueOf(detected.ftcPose.z));

                telemetry.addData("P: ", String.valueOf(detected.ftcPose.pitch));
                telemetry.addData("R: ", String.valueOf(detected.ftcPose.roll));
                telemetry.addData("Y: ", String.valueOf(detected.ftcPose.yaw));
            }
        }
        telemetry.update();

    }
}
