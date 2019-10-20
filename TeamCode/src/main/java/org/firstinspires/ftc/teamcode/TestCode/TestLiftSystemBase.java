package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TestLiftSystemBase extends OpMode {

    private static double LIFT_BASE_POWER = 0.8;

    private DcMotor liftBase;


    public void init(){

        liftBase = hardwareMap.dcMotor.get("liftBase");
    }

    public void loop(){

        if(gamepad1.a){

            liftBase.setPower(LIFT_BASE_POWER);
        }

        if(gamepad1.b){

            liftBase.setPower(-LIFT_BASE_POWER);
        }

        if(gamepad1.dpad_down){
            LIFT_BASE_POWER = LIFT_BASE_POWER - 0.1;

        }

        if(gamepad1.dpad_up){
            LIFT_BASE_POWER = LIFT_BASE_POWER + 0.1;
        }



    }



}
