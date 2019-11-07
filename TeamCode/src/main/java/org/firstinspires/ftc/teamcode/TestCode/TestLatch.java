package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Assemblies.Latch;

@TeleOp(name = "TestLatch")


public class TestLatch extends OpMode {


    //two servos for closing, one for rotating on top

    Latch latch = new Latch(hardwareMap, telemetry);


    public void init() {
        latch.initializeLatch();

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

//        latch.telemetryLatch();
//        telemetry.update();
    }

}
