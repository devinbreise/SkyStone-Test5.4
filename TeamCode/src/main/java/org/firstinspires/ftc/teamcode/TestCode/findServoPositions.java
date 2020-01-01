package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;


@TeleOp(name = "findServoPositions")

public class findServoPositions extends OpMode {
    public static final double MAJOR_INCREMENT = 0.05;
    public static final double MINOR_INCREMENT = 0.01;
    public static double INITIAL_POS = .5;
    public double currentPosition = INITIAL_POS;

    private Servo servo;


    public void init() {
        servo = hardwareMap.servo.get("servo");
        servo.setPosition(INITIAL_POS);
    }

    public void loop() {
        if (gamepad2.b && (currentPosition<1)) {
            currentPosition = currentPosition + MAJOR_INCREMENT;
            servo.setPosition(currentPosition);
            teamUtil.sleep(500);
        }
        if (gamepad2.a && (currentPosition>0)) {
            currentPosition = currentPosition - MAJOR_INCREMENT;
            servo.setPosition(currentPosition);
            teamUtil.sleep(500);

        }
        if (gamepad2.y && (currentPosition<1)) {
            currentPosition = currentPosition + MINOR_INCREMENT;
            servo.setPosition(currentPosition);
            teamUtil.sleep(500);
        }
        if (gamepad2.x && (currentPosition>0)) {
            currentPosition = currentPosition - MINOR_INCREMENT;
            servo.setPosition(currentPosition);
            teamUtil.sleep(500);

        }
        telemetry.addData("position: ", servo.getPosition());
        telemetry.update();
    }
}



