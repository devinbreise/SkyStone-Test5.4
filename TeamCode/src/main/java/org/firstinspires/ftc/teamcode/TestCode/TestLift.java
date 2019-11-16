package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Assemblies.Lift;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "TestLift")
public class TestLift extends OpMode {

    Lift lift;

    public void init() {

        lift = new Lift(hardwareMap, telemetry);
        lift.initLift();
    }

    public void loop() {

        if (gamepad1.a) {

            lift.liftUp();
        } else if (gamepad1.b) {

            lift.liftDown();
        } else lift.shutDownLift();

        if (gamepad1.dpad_down) {
            lift.increaseLiftPower();
        }

        if (gamepad1.dpad_up) {
            lift.decreaseLiftPower();
        }

        lift.liftTelemetry();


    }


}
