package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.Assemblies.Lift;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "TestLiftSystemBase")
public class TestLiftSystemBase extends OpMode {

    private double LIFT_BASE_POWER = 1;

    private Lift lift;

    public void init() {
        lift = new Lift(hardwareMap, telemetry);
        lift.initLift();
    }

    public void loop() {

        // manual limbo
        if (gamepad1.a) {
            lift.liftUp();
        } else if (gamepad1.b) {
            lift.liftDown();
        } else lift.shutDownLift();

        if (gamepad1.dpad_down) {
            lift.decreaseLiftPower();
        }
        if (gamepad1.dpad_up) {
            lift.increaseLiftPower();
        }


        // manual elevator control
        if (gamepad2.a) {
            lift.tensionLiftString();
            while (gamepad2.a) {}
        }
        if (gamepad2.x) {
            lift.goToLevel(0);
            while (gamepad2.x) { }

/*            telemetry.addData("lifting to level 0", 0);
            telemetry.update();
            rSpindle.setTargetPosition(LEVEL_0);
            lSpindle.setTargetPosition(LEVEL_0);
            telemetry.addData("L Target:", lSpindle.getTargetPosition());
            telemetry.addData("R Target:", rSpindle.getTargetPosition());
            rSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.update();
            //while (gamepad2.x){};

            rSpindle.setPower(.99);
            lSpindle.setPower(.99);
            while(lSpindle.isBusy()){
                telemetry.addData("lifting L:", lSpindle.getCurrentPosition());
                telemetry.addData("lifting R:", rSpindle.getCurrentPosition());
                telemetry.update();
                teamUtil.log("lSpindle:"+lSpindle.getCurrentPosition());
           }
*/
        }
        if (gamepad2.y) {
            lift.goToLevel(5);
            while (gamepad2.y) { }
        }
        if (gamepad2.b) {
            lift.goToBottom();
            while (gamepad2.b) { }
        }

//super cereal disclaimer. ts not sponsered its just amazing. or mabye im lieing. mabye this is straight our part of a plan for world domination. "that sounds rather unrealistic" you say with a gulible expression on your face. now let me ask you. what about knees. "knees?" you say with suprise. yes knees. what about them. they are funky looking and all bendy. but you know the ld saying. where there are knees there are bees
        lift.liftTelemetry();
        telemetry.update();
    }
}
