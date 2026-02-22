package org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MidLevel.SwervePod;
import org.firstinspires.ftc.teamcode.subsystems.UpperLevel.SwerveDrivetrain;

import dev.nextftc.ftc.NextFTCOpMode;

@Configurable
@TeleOp(name = "AlignPods", group = "TeleOp")
public class AlignPods extends NextFTCOpMode {
    private static SwerveDrivetrain swerveDrivetrain;
    SwervePod[] pods;

    public static boolean runPower = false;
    public static double target = 0;

    JoinedTelemetry joinedTelemetry;



    @Override
    public void onInit() {
        swerveDrivetrain = new SwerveDrivetrain();
        pods = swerveDrivetrain.getSwervePods();
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    @Override
    public void onUpdate() {
//        swerveDrivetrain.simpleRunDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x);


        if(gamepad1.aWasPressed()) {
            SwerveDrivetrain.flOffset = (int) pods[0].getAnalogInPos().inDeg;
            SwerveDrivetrain.frOffset = (int) pods[1].getAnalogInPos().inDeg;
            SwerveDrivetrain.blOffset = (int) pods[2].getAnalogInPos().inDeg;
            SwerveDrivetrain.brOffset = (int) pods[3].getAnalogInPos().inDeg;
        }


        if (runPower) {
            swerveDrivetrain.updatePods();
//
        } else {
            swerveDrivetrain.updatePodsUnpowered();
        }

        joinedTelemetry.addData("FL currentPos Deg: ", pods[0].getAnalogInPos().inDeg);
        joinedTelemetry.addData("FL currentPos Rad: ", pods[0].getAnalogInPos().inRad);
        joinedTelemetry.addData("FR currentPos Deg: ", pods[1].getAnalogInPos().inDeg);
        joinedTelemetry.addData("FR currentPos Rad: ", pods[1].getAnalogInPos().inRad);
        joinedTelemetry.addData("BL currentPos Deg: ", pods[2].getAnalogInPos().inDeg);
        joinedTelemetry.addData("BL currentPos Rad: ", pods[2].getAnalogInPos().inRad);
        joinedTelemetry.addData("BR currentPos Deg: ", pods[3].getAnalogInPos().inDeg);
        joinedTelemetry.addData("BR currentPos Rad: ", pods[3].getAnalogInPos().inRad);

        joinedTelemetry.addData("current FL: ", SwerveDrivetrain.flOffset);
        joinedTelemetry.addData("current FR: ", SwerveDrivetrain.frOffset);
        joinedTelemetry.addData("current BL: ", SwerveDrivetrain.blOffset);
        joinedTelemetry.addData("current BR: ", SwerveDrivetrain.brOffset);

        joinedTelemetry.addLine(swerveDrivetrain.debugText());
        joinedTelemetry.update();
    }
}
