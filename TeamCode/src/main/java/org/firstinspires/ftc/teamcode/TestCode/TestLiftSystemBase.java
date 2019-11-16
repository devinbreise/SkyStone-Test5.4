package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "TestLiftSystemBase")
public class TestLiftSystemBase extends OpMode {

    private double LIFT_BASE_POWER = 1;

    private DcMotor liftBase;
    private DcMotor rSpindle;
    private DcMotor lSpindle;
    private final int LEVEL_0 = 340;
    private final int LEVEL_INCREMENT = 570;


    public void init() {

        liftBase = hardwareMap.dcMotor.get("liftBase");
        rSpindle = hardwareMap.dcMotor.get("rSpindle");
        lSpindle = hardwareMap.dcMotor.get("lSpindle");
        lSpindle.setDirection(DcMotorSimple.Direction.REVERSE);


        LIFT_BASE_POWER = 1;
    }

    public void loop() {

        if (gamepad1.a) {

            liftBase.setPower(LIFT_BASE_POWER);
        } else if (gamepad1.b) {

            liftBase.setPower(-LIFT_BASE_POWER);
        } else liftBase.setPower(0);

        if (gamepad1.dpad_down) {
            LIFT_BASE_POWER = LIFT_BASE_POWER - 0.1;
            teamUtil.sleep(500);
        }

        if (gamepad1.dpad_up) {
            LIFT_BASE_POWER = LIFT_BASE_POWER + 0.1;
            teamUtil.sleep(500);
        }
        if (gamepad2.a) {
            rSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rSpindle.setPower(0.05);
            lSpindle.setPower(0.05);
            teamUtil.sleep(2000);
            rSpindle.setPower(0);
            lSpindle.setPower(0);
            rSpindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lSpindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (gamepad2.x) {
            telemetry.addData("lifting to level 0", 0);
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
/*            while(lSpindle.isBusy()){
                telemetry.addData("lifting L:", lSpindle.getCurrentPosition());
                telemetry.addData("lifting R:", rSpindle.getCurrentPosition());
                telemetry.update();
                teamUtil.log("lSpindle:"+lSpindle.getCurrentPosition());
           }*/

            if (gamepad2.y) {
                rSpindle.setTargetPosition(LEVEL_0 + LEVEL_INCREMENT * 6);
                lSpindle.setTargetPosition(LEVEL_0 + LEVEL_INCREMENT * 6);
                rSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rSpindle.setPower(.99);
                lSpindle.setPower(.99);
            }

//super cereal disclaimer. ts not sponsered its just amazing. or mabye im lieing. mabye this is straight our part of a plan for world domination. "that sounds rather unrealistic" you say with a gulible expression on your face. now let me ask you. what about knees. "knees?" you say with suprise. yes knees. what about them. they are funky looking and all bendy. but you know the ld saying. where there are knees there are bees
            telemetry.addData("Motor Power:", LIFT_BASE_POWER);
            telemetry.addData("lSpindle:", lSpindle.getCurrentPosition());
            telemetry.addData("rSpindle:", rSpindle.getCurrentPosition());
            telemetry.update();
        }


    }
}
