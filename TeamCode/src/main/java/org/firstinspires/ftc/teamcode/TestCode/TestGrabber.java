
package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;

@TeleOp(name = "TestGrabber")
@Disabled

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
        if (gamepad2.b) { //close wide

            grabber.grabberStow();
        }
        if (gamepad2.x) { //close narrow
            grabber.closeGrabberWide();
        }
        if (gamepad2.a){
            grabber.grabberRotatePos();
        }
        if(gamepad2.dpad_up){
            grabber.rotate(Grabber.GrabberRotation.OUTSIDE);
        }
        if(gamepad2.dpad_down){
           // grabber.rotateSetPosition(grabber.rotateGetPosition()-0.01);
            grabber.rotate(Grabber.GrabberRotation.INSIDE);
        }
        if(gamepad2.dpad_right){
            grabber.rotate(Grabber.GrabberRotation.MIDDLE);
        }


        grabber.grabberTelemetry();

    }
}



