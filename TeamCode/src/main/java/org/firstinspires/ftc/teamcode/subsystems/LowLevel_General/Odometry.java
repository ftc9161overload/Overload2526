package org.firstinspires.ftc.teamcode.subsystems.LowLevel_General;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class Odometry implements Subsystem {



    private GoBildaPinpointDriver pinpointDriver;

    public static final Odometry INSTANCE = new Odometry();


    private Odometry() {}

    @Override
    public void initialize() {
        pinpointDriver = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
    }

    public void initReal() {
        pinpointDriver = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public Command reset = new InstantCommand(() -> {
        pinpointDriver.resetPosAndIMU();
    });

    public Pose2D getPos() {
        return pinpointDriver.getPosition();
    }

    public double getX() {return pinpointDriver.getPosX(DistanceUnit.INCH);}
    public double getY() {return pinpointDriver.getPosY(DistanceUnit.INCH);}
    public double getHeading() { return pinpointDriver.getHeading(AngleUnit.RADIANS);}




    @Override
    public void periodic() {
        pinpointDriver.update();
    }


}
