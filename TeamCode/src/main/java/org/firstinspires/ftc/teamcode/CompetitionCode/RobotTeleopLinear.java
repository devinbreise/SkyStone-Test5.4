package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.LiftSystem;
import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "RobotTeleopLinear")
public class RobotTeleopLinear extends LinearOpMode {
    public static final double SCALE_DOWN_CONSTANT = 0.3;
    int level = 0;
    Grabber.GrabberRotation grabberRotation;
    TeamGamepad teamGamePad;

    Robot robot;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT);
        robot = new Robot(this);

        teamGamePad = new TeamGamepad(this);

        robot.init();
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.READY_TO_START);
        teamUtil.initPerf();

    }

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addLine("Initializing Op Mode...please wait");
        telemetry.update();
        initialize();
        grabberRotation = Grabber.GrabberRotation.INSIDE;

        telemetry.addLine("Ready to Stack :D");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            //robot.scaleMovement(MAX_POWER, DRIVE_POWER);
            telemetry.addData("Heading:", robot.drive.getHeading());
///////////////////////////////////////////////////////////////////////
            teamGamePad.gamepadLoop();


///////////////////////////////////////////////////////////////////////
            //this code is for the drive
            if (gamepad1.left_trigger > 0.5) {
                robot.drive.universalJoystick(gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x, 1,
                        robot.drive.getHeading());

            } else {
                robot.drive.universalJoystick(gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x, SCALE_DOWN_CONSTANT,
                        robot.drive.getHeading());

            }


            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                robot.drive.resetHeading();
            }
/////////////////////////////////////////////////////////////////////////

            if (gamepad1.right_bumper  && gamepad1.right_trigger < 0.5) {
                robot.autoIntake(6500);
            }

            if (gamepad1.right_bumper && gamepad1.right_trigger > 0.5) {
                robot.liftSystem.grab("wide", 4000);
            }

            if (gamepad1.left_bumper && gamepad1.right_trigger > 0.5) {
                robot.liftSystem.grabAndStowNoWait("wide", 4000);
            }

            if(gamepad1.x){
                robot.drive.moveToPickUpDistance(4000);
            }

            if(gamepad1.b){
                robot.drive.moveToPickUpDistance(4000);
                robot.drive.moveInchesForward(0.15, 0.5, 2000);
            }

            if(gamepad1.a){
                robot.liftSystem.grab("narrow", 4000);
            }

/////////////////////////////////////////////////////////////////////////
            //this code is for the foundation latch
            if (gamepad1.dpad_down) {
                robot.latch.latchDown();

            } else if (gamepad1.dpad_up) {
                robot.latch.latchUp();
            }

            if (gamepad1.dpad_left || gamepad1.dpad_right) {
                robot.latch.latchMiddle();
            }
/////////////////////////////////////////////////////////////////////
            //this code is for the lift system assembly

            if (teamGamePad.gamepad2dpad_upBounced()) {
                level += 1;
                teamUtil.log("Level Up :D *cue final fantasy music");
            }
            if (teamGamePad.gamepad2dpad_downBounced()) {
                level -= 1;
                teamUtil.log("Level decreased :C");
            }
            if (robot.liftSystem.state == LiftSystem.LiftSystemState.HOVER) {
                robot.liftSystem.lift.moveElevatorToLevelNoWait(level, 4500);
            }


//            if (gamepad2.y) {
//                grabberRotation = Grabber.GrabberRotation.OUTSIDE;
//            }
//            if (gamepad2.b) {
//                grabberRotation = Grabber.GrabberRotation.MIDDLE;
//            }
//            if (gamepad2.a) {
//                grabberRotation = Grabber.GrabberRotation.INSIDE;
//            }

            if (gamepad2.right_bumper && gamepad2.right_trigger > 0.5) {
                robot.autoDropOffRight(level, 7000);
            }

            if (gamepad2.left_bumper && gamepad2.right_trigger > 0.5) {
                robot.autoDropOffLeft(level, 7000);
            }


            if (gamepad2.right_bumper && gamepad2.right_trigger < 0.5) {
                robot.liftSystem.hoverOverFoundation(level, grabberRotation, 7000);
                teamUtil.log("tried to deploy");
                teamUtil.log("level: " + level);
            }

            if (gamepad2.left_bumper && gamepad2.right_trigger < 0.5) {
                robot.liftSystem.prepareToGrabNoWait(9000);
            }
            if (gamepad2.right_stick_button) {
                robot.liftSystem.drop();
            }
            if (teamGamePad.gamepad2XBounced()) {

                robot.liftSystem.liftDown();
            }
            if (gamepad2.a) {
                robot.liftSystem.putAwayLiftSystemNoWait(5000);
            }


/////////////////////////////////////////////////////////////////////
            //this code is the telemetry
//        latch.latchTelemetry();
//        robot.driveTelemetry();
            telemetry.addData("level:", level);
            telemetry.addData("grabber rotation", grabberRotation);
            telemetry.addData("frontMiddleDistance", robot.drive.frontmiddleDistance.getDistance(DistanceUnit.CM));
            telemetry.update();

            // teamUtil.trackPerf();

            if (level == 0) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.CURIOSITY);
            } else if (level == 1) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LEVEL_0);

            } else if (level == 2) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LEVEL_1);

            } else if (level == 3) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LEVEL_2);

            } else if (level == 4) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LEVEL_3);

            } else if (level == 5) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LEVEL_4);

            } else if (level == 6) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LEVEL_5);

            }


        }
    }

}
