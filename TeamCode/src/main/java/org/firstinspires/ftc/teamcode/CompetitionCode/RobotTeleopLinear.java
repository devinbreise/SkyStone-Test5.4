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

@TeleOp(name = "RobotTeleopLinear", group ="Competition")
public class RobotTeleopLinear extends LinearOpMode {
    public static final double SCALE_DOWN_CONSTANT = 0.3;
    int level = 0;
    Grabber.GrabberRotation grabberRotation;
    TeamGamepad teamGamePad;

    Robot robot;

    public void initialize() {

        teamUtil.init(this);
        teamUtil.telemetry.addLine("Initializing Op Mode...please wait");
        teamUtil.telemetry.update();
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT);

        robot = new Robot(this);

        teamGamePad = new TeamGamepad(this);

        robot.init(false);
        robot.latch.latchUp();
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.READY_TO_START);
        teamUtil.initPerf();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        grabberRotation = Grabber.GrabberRotation.INSIDE;

        teamUtil.telemetry.addLine("Ready to Stack :D");
        teamUtil.telemetry.update();
        waitForStart();
        robot.latch.latchUp(); // move latches up at start of teleop

        while (opModeIsActive()) {

            //robot.scaleMovement(MAX_POWER, DRIVE_POWER);
            teamUtil.telemetry.addData("Heading:", robot.drive.getHeading());
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

            // Disable these 3 commands unless the lift system is in the 'prepare to grab' position
            // this might keep a critical mistake from happening (e.g. dropping the lift on top of a tower)
            if ((robot.liftSystem.state == LiftSystem.LiftSystemState.IDLE) || robot.liftSystem.preparedToGrab) {
                if (gamepad1.right_bumper) {
                    robot.liftSystem.grabNoWait( 4000);

                }
                if(gamepad1.y){
                     robot.liftSystem.capAndGrabNoWait(5000);
                }

                if (gamepad1.left_bumper) {
                    robot.liftSystem.grabAndStowNoWait(4000);
                }
            }

/////////////////////////////////////////////////////////////////////////
            //this code is for the foundation latch
            if (gamepad1.dpad_down) {
                robot.latch.latchDown();

            } else if (gamepad1.dpad_up) {
                robot.latch.latchUp();
            }

            if (gamepad1.dpad_left) {
                robot.latch.latchMiddle();
            }

            if (gamepad1.dpad_right) {
                robot.latch.latchPushbot();
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


            if (gamepad2.right_bumper && gamepad2.right_trigger > 0.5) {
                robot.liftSystem.putAwayLiftSystemNoWait(5000);
            }



            if (gamepad2.right_bumper && gamepad2.right_trigger < 0.5) {
                robot.liftSystem.hoverOverFoundationNoWait(level, grabberRotation, 7000);
                teamUtil.log("tried to deploy");
                teamUtil.log("level: " + level);
            }

            if (gamepad2.left_bumper && gamepad2.left_trigger > 0.5 && gamepad2.right_trigger < 0.5) {
                robot.liftSystem.prepareToGrabNoWait(9000, Grabber.GrabberRotation.INSIDE);
            }

            if(gamepad2.left_bumper && gamepad2.left_trigger > 0.5 && gamepad2.right_trigger > 0.5){
                robot.liftSystem.prepareToGrabNoWait(9000, Grabber.GrabberRotation.MIDDLE);
            }

            if (gamepad2.right_stick_button) {
                robot.liftSystem.drop();
            }
            if (teamGamePad.gamepad2ABounced()) {
                robot.liftSystem.elevatorDown();
            }
            if (gamepad2.y) {
                robot.liftSystem.grabber.dropCapstone();
            }
            if(gamepad2.b){
                robot.liftSystem.hoverOverFoundationNoWait(level, Grabber.GrabberRotation.MIDDLE, 7000);
            }

            if (gamepad2.left_stick_y < -0.8) {
                robot.liftSystem.lift.moveElevatorUpSlowly();
            } else  if (gamepad2.left_stick_y > 0.8) {
                robot.liftSystem.lift.moveElevatorDownSlowly();
            } else {
                robot.liftSystem.lift.manualHoldElevator(); // this will stop any other elevator movement dead in its tracks...
            }



/////////////////////////////////////////////////////////////////////
            //this code is the telemetry
//        latch.latchTelemetry();
//        robot.driveTelemetry();
            teamUtil.telemetry.addData("level:", level);
            teamUtil.telemetry.addData("grabber rotation", grabberRotation);
//            teamUtil.telemetry.addData("frontMiddleDistance", robot.drive.frontmiddleDistance.getDistance(DistanceUnit.CM));
            teamUtil.telemetry.update();

            // teamUtil.trackPerf();

            if (level == 0) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LEVEL_0);
            } else if (level == 1) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LEVEL_1);

            } else if (level == 2) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LEVEL_2);

            } else if (level == 3) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LEVEL_3);

            } else if (level == 4) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LEVEL_4);

            } else if (level == 5) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LEVEL_5);

            } else if (level == 6) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LEVEL_6);

            }


        }
    }

}
