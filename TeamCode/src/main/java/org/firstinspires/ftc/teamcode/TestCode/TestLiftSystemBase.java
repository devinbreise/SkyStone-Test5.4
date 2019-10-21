package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TestLiftSystemBase")
public class TestLiftSystemBase extends OpMode {

    private static double LIFT_BASE_POWER = 0.8;

    private DcMotor liftBase;


    public void init() {

        liftBase = hardwareMap.dcMotor.get("liftBase");
    }

    public void loop() {

        if (gamepad1.a) {

            liftBase.setPower(LIFT_BASE_POWER);
        } else if (gamepad1.b) {

            liftBase.setPower(-LIFT_BASE_POWER);
        } else liftBase.setPower(0);

        if (gamepad1.dpad_down) {
            LIFT_BASE_POWER = LIFT_BASE_POWER - 0.1;

        }

        if (gamepad1.dpad_up) {
            LIFT_BASE_POWER = LIFT_BASE_POWER + 0.1;
        }


    }


}
