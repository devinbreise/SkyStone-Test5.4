
package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestGrabberIdea2")
@Disabled
public class TestGrabberIdea2 extends OpMode {
    public static final double ROTATE_GRABBER_INCREMENT = 0.1;


    //two servos for closing, one for rotating on top

    public static double GRABBER_ONE_OPEN_POS = 1;
    public static double GRABBER_ONE_WIDE_CLOSED_POS = 0.64;
    public static double GRABBER_ONE_NARROW_CLOSED_POS = 0.45;

    public static double GRABBER_TWO_OPEN_POS = 0;
    public static double GRABBER_TWO_WIDE_CLOSED_POS = 0.33;
    public static double GRABBER_TWO_NARROW_CLOSED_POS = 0.52;

    private Servo grabberOne;
    private Servo grabberTwo;
    //private Servo rotateGrabber;


        public void init() {
            grabberOne = hardwareMap.servo.get("grabberOne");
            grabberTwo = hardwareMap.servo.get("grabberTwo");
            //rotateGrabber = hardwareMap.servo.get("rotateGrabber");


            grabberOne.setPosition(GRABBER_ONE_OPEN_POS);
            grabberTwo.setPosition(GRABBER_TWO_OPEN_POS);
            //CURRENT_POS = ROTATE_GRABBER_INITIAL_POS;
            //rotateGrabber.setPosition(CURRENT_POS);
        }

        public void loop() {
            if (gamepad2.y) { //open
                grabberOne.setPosition(GRABBER_ONE_OPEN_POS);
                grabberTwo.setPosition(GRABBER_TWO_OPEN_POS);
                telemetry.addData("GrabberOne: ", grabberOne.getPosition());
                telemetry.addData("GrabberTwo: ", grabberTwo.getPosition());
            }
            if (gamepad2.x) { //close wide
                grabberOne.setPosition(GRABBER_ONE_WIDE_CLOSED_POS);
                grabberTwo.setPosition(GRABBER_TWO_WIDE_CLOSED_POS);
                telemetry.addData("GrabberOne: ", grabberOne.getPosition());
                telemetry.addData("GrabberTwo: ", grabberTwo.getPosition());
            }
            if (gamepad2.a) { //close narrow
                grabberOne.setPosition(GRABBER_ONE_NARROW_CLOSED_POS);
                grabberTwo.setPosition(GRABBER_TWO_NARROW_CLOSED_POS);
                telemetry.addData("GrabberOne: ", grabberOne.getPosition());
                telemetry.addData("GrabberTwo: ", grabberTwo.getPosition());
            }

 /*
            if (gamepad2.x) {
                double position = CURRENT_POS + ROTATE_GRABBER_INCREMENT;
                rotateGrabber.setPosition(position);
                CURRENT_POS = position;
                telemetry.addData("rotateGrabber: ", rotateGrabber.getPosition());

            }

            if (gamepad2.y) {
                double position = CURRENT_POS - ROTATE_GRABBER_INCREMENT;
                rotateGrabber.setPosition(position);
                CURRENT_POS = position;
                telemetry.addData("rotateGrabber: ", rotateGrabber.getPosition());

            }
*/

            telemetry.update();


        }
    }



