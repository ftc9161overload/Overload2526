package org.firstinspires.ftc.teamcode.subsystems.LowLevel_General;

import android.util.Size;

import com.bylazar.telemetry.JoinedTelemetry;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Camera implements Subsystem {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private JoinedTelemetry telemetry;
    private UniConstants.loggingState state = UniConstants.loggingState.ENABLED;
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();
    private ArrayList<Integer> detectionIDs = new ArrayList<>();
    private ArrayList<UniConstants.slotState> pattern = new ArrayList<>();
    private double distanceToGoal = 0;
    private double deltaAngle = 0;

    public static final Camera INSTANCE = new Camera();

    private Timer timer = new Timer();

//    public BetterVisionTM() {}




    @Override
    public void initialize(){

        //telemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());

        //21 - 23 for obelisk
        //20 for blue goal
        //24 for red goal

        // Build AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // Build Vision Portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(ActiveOpMode.hardwareMap().get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();
    }



    /** Returns the full list of current AprilTag detections. */
    public void getDetections() {
        detections = aprilTagProcessor.getDetections();
        detectionIDs = getDetectionIDs();
    }





    public void analyzeTags() {
        for(AprilTagDetection tagData : detections){
            if (tagData != null) {
                // Logic to determine position based on tag ID

                double x = tagData.ftcPose.x;
                double y = tagData.ftcPose.y;
                double dist = tagData.ftcPose.range;
                double yaw = tagData.ftcPose.yaw;
                double pitch = tagData.ftcPose.pitch;
                double realYaw = Math.toDegrees(Math.atan2(x,y));

                double hypotenuse = Math.hypot(x, y);

//                if((color == UniConstants.teamColor.BLUE && tagData.id == 20) || (color == UniConstants.teamColor.RED && tagData.id == 24)){
//                    distanceToGoal = dist;
//                }


                if (state == UniConstants.loggingState.ENABLED) {
                    telemetry.addData("Tag ID: ", tagData.id);
                    telemetry.addData("X: ", x);
                    telemetry.addData("Y: ", y);
                    telemetry.update();
                }
                else if (state == UniConstants.loggingState.EXTREME) {
                    telemetry.addData("Tag ID: ", tagData.id);
                    telemetry.addData("X: ", x);
                    telemetry.addData("Y: ", y);
                    telemetry.addData("Pitch: ", pitch);
                    telemetry.addData("Yaw: ", yaw);
                    telemetry.addData("Real Yaw: ", realYaw);
                    telemetry.addData("Hypotenuse: ", hypotenuse);
                    telemetry.update();
                }

            }
        }
    }

    public void quickAnalyzeGoal(){
        getDetections();
        if(!detections.contains(null)){
            for(AprilTagDetection detection : detections){
                double dist = detection.ftcPose.range;
                double pitch = detection.ftcPose.pitch;

//                if((color == UniConstants.teamColor.BLUE && detection.id == 20) || (color == UniConstants.teamColor.RED && detection.id == 24)){
//                    distanceToGoal = dist;
//                    deltaAngle = pitch;
//                }

            }
        }}

    public ArrayList<Integer> getDetectionIDs(){
        ArrayList<Integer> ids = new ArrayList<>();
        for (AprilTagDetection detection : detections){
            ids.add(detection.id);
        }
        return ids;
    }

    public int getObeliskID() {
        if(detectionIDs.contains(21) || detectionIDs.contains(22) || detectionIDs.contains(23)){
            for(int id : detectionIDs){
                if ((id == 21) || (id == 22) || (id == 23)){
                    return id;
                }
            }
        }
        return -1;
    }

    public ArrayList<UniConstants.slotState> getPattern(){
        obeliskTargetPattern(getObeliskID());
        return pattern;
    }

    /** Stops the vision portal */
    public void stop() {
        visionPortal.close();
    }

    @Override
    public void periodic() {

//        if(!Robot.patternFull){
//            getDetections();
//        }

    }

    public double getDistanceToGoal(){
        return distanceToGoal;
    }

    public double getDeltaAngle(){
        return deltaAngle;
    }

    public void obeliskTargetPattern(int ID){
        switch(ID){
            case -1:
                pattern = new ArrayList<>(List.of(null, null, null));
                break;
            case 21:
                pattern = new ArrayList<>(List.of(UniConstants.slotState.GREEN, UniConstants.slotState.PURPLE, UniConstants.slotState.PURPLE));
                break;
            case 22:
                pattern = new ArrayList<>(List.of(UniConstants.slotState.PURPLE, UniConstants.slotState.GREEN, UniConstants.slotState.PURPLE));
                break;
            case 23:
                pattern = new ArrayList<>(List.of(UniConstants.slotState.PURPLE, UniConstants.slotState.PURPLE, UniConstants.slotState.GREEN));
                break;
        }

    }

//    public void setColor(UniConstants.teamColor col){color = col;}

    public void setTelemetry(JoinedTelemetry telemetry){
        this.telemetry = telemetry;
    }
}