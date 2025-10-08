package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import dev.nextftc.core.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;

public class IntakeSubystem implements Subsystem {
    private boolean intakeOn;
    private final Telemetry telemetry;

    // Code to create array lists storing the motors and servos -
    // can be changed if/when we make a motor and servo subsystem
    private final ArrayList<DcMotor> motors = new ArrayList<>();
    private final ArrayList<Servo> servos = new ArrayList<>();

    public IntakeSubystem(Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeOn = false;

    }

    // Sets whether or not the intake is on
    public void setIntakeOn(boolean intakeOn) {
        this.intakeOn = intakeOn;
    }

    // Adds a motor to the motor array list variable
    public void addMotor(DcMotor motor) {
        motors.add(motor);
    }

    // Adds a servo to the servo array list variable
    public void addServo(Servo servo) {
        servos.add(servo);
    }

    // Method which checks the motor power and servo position for each servo and motor
    public void servoAndMotorPower() {
        for (int i = 0; i < motors.size(); i++) {
            DcMotor motor = motors.get(i);
            telemetry.addData("Motor number: " + i + " Power", motor.getPower());
        }
        for (int i = 0; i < servos.size(); i++) {
            Servo servo = servos.get(i);
            telemetry.addData("Servo number: " + i + " Pos", servo.getPosition());
        }
        telemetry.update();
    }



}
