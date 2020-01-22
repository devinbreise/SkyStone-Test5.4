
package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "TestGrabber")

public class TestGrabber extends LinearOpMode {

    Grabber grabber;
    TeamGamepad teamGamepad;

    @Override
    public void runOpMode() throws InterruptedException {

        teamUtil.init(this);
        grabber = new Grabber(hardwareMap, teamUtil.telemetry);
        teamGamepad = new TeamGamepad(this);
        grabber.initGrabber();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.y) { //open
                grabber.openGrabber();
            }
            if (gamepad2.b) { //close wide
                grabber.narrowOpen();
            }
            if (gamepad2.x) { //close narrow
                grabber.closeGrabberWide();
            }
            if (gamepad2.a) {
            grabber.grabberRotatePos();
//                grabber.slightlyOpenGrabber();
            }
            if (gamepad2.dpad_up) {
                grabber.rotate(Grabber.GrabberRotation.OUTSIDE);
            }
            if (gamepad2.dpad_down) {
                // grabber.rotateSetPosition(grabber.rotateGetPosition()-0.01);
                grabber.rotate(Grabber.GrabberRotation.INSIDE);
            }
            if (gamepad2.dpad_right) {
                grabber.rotate(Grabber.GrabberRotation.MIDDLE);
            }


            grabber.grabberTelemetry();

        }
    }
}




