package org.firstinspires.ftc.teamcode.TestCode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestGrabberIdea2")
@Disabled

public class TestGrabberIdea2 extends LinearOpMode {
    public static final double ROTATE_GRABBER_INCREMENT = 0.1;


    //two servos for closing, one for rotating on top

    public static double GRABBER_ONE_CLOSED_POS = 0;
    public static double GRABBER_TWO_CLOSED_POS = 1;

    public static double GRABBER_ONE_OPEN_POS = 0.5;
    public static double GRABBER_TWO_OPEN_POS = 0.5;
    public static int ROTATE_GRABBER_INITIAL_POS = 0;


    private Servo grabberOne;
    private Servo grabberTwo;
    private Servo rotateGrabber;

    @Override
    public void runOpMode() throws InterruptedException {


        grabberOne = hardwareMap.servo.get("grabberOne");
        grabberTwo = hardwareMap.servo.get("grabberTwo");
//        rotateGrabber = hardwareMap.servo.get("rotateGrabber");


        grabberOne.setPosition(GRABBER_ONE_OPEN_POS);
        grabberTwo.setPosition(GRABBER_TWO_OPEN_POS);
//        rotateGrabber.setPosition(ROTATE_GRABBER_INITIAL_POS);


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.a) {
                grabberOne.setPosition(GRABBER_ONE_CLOSED_POS);
                grabberTwo.setPosition(GRABBER_TWO_CLOSED_POS);
                telemetry.addData("GrabberOne: ", grabberOne.getPosition());
                telemetry.addData("GrabberTwo: ", grabberTwo.getPosition());

                //close
            }

            if (gamepad2.b) {
                grabberOne.setPosition(GRABBER_ONE_OPEN_POS);
                grabberTwo.setPosition(GRABBER_TWO_OPEN_POS);
                telemetry.addData("GrabberOne: ", grabberOne.getPosition());
                telemetry.addData("GrabberTwo: ", grabberTwo.getPosition());


                //open
            }

//            if (gamepad2.x) {
//                double position = ROTATE_GRABBER_INITIAL_POS + ROTATE_GRABBER_INCREMENT;
//                rotateGrabber.setPosition(position);
//                telemetry.addData("rotateGrabber: ", rotateGrabber.getPosition());
//
//            }

            telemetry.update();


        }
    }


}
