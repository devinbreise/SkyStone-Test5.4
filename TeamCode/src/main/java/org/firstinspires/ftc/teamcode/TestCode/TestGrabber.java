
package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;

@TeleOp(name = "TestGrabber")

public class TestGrabber extends OpMode {

    Grabber grabber;
    TeamGamepad teamGamepad;

    public void init() {
        grabber = new Grabber(hardwareMap, telemetry);
        teamGamepad = new TeamGamepad(this);
        grabber.initGrabber();
    }

    public void loop() {
        if (gamepad2.y) { //open
            grabber.openGrabber();
        }
        if (teamGamepad.gamepad2bBounced()) { //close wide

            grabber.closeGrabberToggle();
        }
        if (gamepad2.x) { //close narrow
            grabber.closeGrabberWide();
        }
        if(gamepad2.dpad_up){
            grabber.rotateOutside();
        }
        if(gamepad2.dpad_down){
            grabber.rotateSetPosition(grabber.rotateGetPosition()-0.01);
          grabber.rotateInside();
        }
        if(gamepad2.dpad_right){
            grabber.rotateNarrow();
        }


        grabber.grabberTelemetry();

    }
}



