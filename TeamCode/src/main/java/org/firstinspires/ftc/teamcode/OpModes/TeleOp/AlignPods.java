package org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MidLevel.SwervePodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.SwerveDrivetrain;

@Configurable
@TeleOp(name = "AlignPods", group = "TeleOp")
public class AlignPods extends OpMode {
    private static SwerveDrivetrain swerveDrivetrain;
    SwervePodSubsystem[] pods;
    public static double p = 0,d = 0,f = 0,l = 0;
    public static boolean runPDFL = false;
    public static double target = 0;

    private JoinedTelemetry joinedTelemetry;

    @Override
    public void init() {
        swerveDrivetrain = new SwerveDrivetrain(hardwareMap);
        pods = swerveDrivetrain.getSwervePods();
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    @Override
    public void loop() {
//        swerveDrivetrain.simpleRunDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x);


        if(gamepad1.aWasPressed()) {
            SwerveDrivetrain.flOffset = (int) pods[0].getAnalogInPos();
            SwerveDrivetrain.frOffset = (int) pods[1].getAnalogInPos();
            SwerveDrivetrain.blOffset = (int) pods[2].getAnalogInPos();
            SwerveDrivetrain.brOffset = (int) pods[3].getAnalogInPos();
        }

        swerveDrivetrain.setPDFLs(p,d,f,l);
        swerveDrivetrain.updatePods();
        if (runPDFL) {
            swerveDrivetrain.setPos(target);
        } else {
//            swerveDrivetrain.setPosZero();
//            swerveDrivetrain.setServoPowZero();
        }

        joinedTelemetry.addData("FL currentPos: ", Math.toDegrees(pods[0].getAnalogInPos()));
        joinedTelemetry.addData("FR currentPos: ", Math.toDegrees(pods[1].getAnalogInPos()));
        joinedTelemetry.addData("BL currentPos: ", Math.toDegrees(pods[2].getAnalogInPos()));
        joinedTelemetry.addData("FR currentPos: ", Math.toDegrees(pods[3].getAnalogInPos()));

        joinedTelemetry.addData("current FL: ", SwerveDrivetrain.flOffset);
        joinedTelemetry.addData("current FR: ", SwerveDrivetrain.frOffset);
        joinedTelemetry.addData("current BL: ", SwerveDrivetrain.blOffset);
        joinedTelemetry.addData("current BR: ", SwerveDrivetrain.brOffset);

        joinedTelemetry.addLine(swerveDrivetrain.debugString());
        joinedTelemetry.update();
    }
}
