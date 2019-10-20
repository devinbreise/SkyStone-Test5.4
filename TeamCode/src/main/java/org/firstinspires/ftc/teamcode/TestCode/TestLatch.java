package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestLatch")
@Disabled

public class TestLatch extends OpMode {


    //two servos for closing, one for rotating on top

    public static double GRABBER_ONE_CLOSED_POS = 0.35;
    public static double GRABBER_TWO_CLOSED_POS = 0.55;

    public static double GRABBER_ONE_OPEN_POS = 0.65;
    public static double GRABBER_TWO_OPEN_POS = 0.35;



    private Servo grabberOne;
    private Servo grabberTwo;

//DIGITAL SERVOS CAN ONLY GO FROM 0.3 AND 0.7!!!!!!!!!!!
            public void init() {
        grabberOne = hardwareMap.get(Servo.class,"grabberOne");
        grabberTwo = hardwareMap.get(Servo.class,"grabberTwo");


        grabberOne.setPosition(GRABBER_ONE_OPEN_POS);
        grabberTwo.setPosition(GRABBER_TWO_OPEN_POS);

    }

    public void loop() {
        //close
        if (gamepad2.a) {
            grabberOne.setPosition(GRABBER_ONE_CLOSED_POS);
            grabberTwo.setPosition(GRABBER_TWO_CLOSED_POS);
            telemetry.addData("GrabberOne: ", grabberOne.getPosition());
            telemetry.addData("GrabberTwo: ", grabberTwo.getPosition());


        }

        //open
        if (gamepad2.b) {
            grabberOne.setPosition(GRABBER_ONE_OPEN_POS);
            grabberTwo.setPosition(GRABBER_TWO_OPEN_POS);
            telemetry.addData("GrabberOne: ", grabberOne.getPosition());
            telemetry.addData("GrabberTwo: ", grabberTwo.getPosition());


        }


        telemetry.update();
    }

}
