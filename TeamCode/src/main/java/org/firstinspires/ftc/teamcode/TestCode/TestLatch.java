package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Latch;

@TeleOp(name = "TestLatch")
@Disabled


public class TestLatch extends OpMode {


    //two servos for closing, one for rotating on top
    Latch latch;




    public void init() {
        latch = new Latch(hardwareMap, telemetry);


        latch.initLatch();


    }

    public void loop() {
        //close
        if (gamepad2.a) {
            latch.latchUp();

        }
        if(gamepad2.x){
            latch.latchDown();
        }

        //open
        if (gamepad2.b) {
            latch.toggleLatch();

        }
        if (gamepad2.y) {
            latch.latchMiddle();

        }

//        latch.latchTelemetry();
//        telemetry.update();
    }

}
