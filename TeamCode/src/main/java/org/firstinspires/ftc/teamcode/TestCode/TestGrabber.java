
package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;

@TeleOp(name = "TestGrabber")
@Disabled
public class TestGrabber extends OpMode {

    Grabber grabber;

    public void init() {
        grabber = new Grabber(hardwareMap, telemetry);
        grabber.initGrabber();
    }

    public void loop() {
        if (gamepad2.y) { //open
            grabber.openGrabber();
        }
        if (gamepad2.b) { //close wide
            grabber.closeGrabberToggle();
        }
        if (gamepad2.x) { //close narrow
            grabber.closeGrabberWide();
        }
        if (gamepad2.y) { //close narrow
            grabber.closeGrabberNarrow();
        }

        grabber.grabberTelemetry();

    }
}



