package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.LiftSystem;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "TestLift")
@Disabled

public class TestLift extends LinearOpMode {

    private double LIFT_BASE_POWER = 1;
    private int targetLevel = 0;

    //private Lift lift;
    private LiftSystem liftSystem;

    public void initialize() {
        teamUtil.init(this);
        liftSystem = new LiftSystem(hardwareMap, telemetry);
        liftSystem.initLiftSystem();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Op Mode...please wait");
        telemetry.update();
        initialize();

        telemetry.addLine("Ready to Test Lift System");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // manual limbo
            /*
            if (gamepad1.b) {
                liftSystem.lift.liftBaseUp();
            } else if (gamepad1.a) {
                liftSystem.lift.liftBaseDown();
            } else liftSystem.lift.shutDownLiftBase();
*/
            if (gamepad1.dpad_down) {
                liftSystem.grabber.closeGrabberWide();
            }
            if (gamepad1.dpad_up) {
                liftSystem.grabber.openGrabber();
            }
            if (gamepad1.dpad_left) {
                targetLevel--;
                teamUtil.sleep(500);
            }
            if (gamepad1.dpad_right) {
                targetLevel++;
                teamUtil.sleep(500);
            }
            if (gamepad1.right_bumper) {
                liftSystem.lift.moveLiftBaseUp(0.99, 7000);
            }
            if (gamepad1.left_bumper) {
                liftSystem.lift.moveLiftBaseDown(0.3, 7000);
            }

            // manual elevator control
            if (gamepad1.a) {
                liftSystem.lift.moveElevator(liftSystem.lift.HOVER_FOR_GRAB, 5000);
            }
            if (gamepad1.b) {
                liftSystem.lift.moveElevator(liftSystem.lift.SAFE_TO_ROTATE, 5000);
            }
            if (gamepad1.x) {
                liftSystem.lift.moveElevatorToBottom();
                //while (gamepad2.a) {};
            }
            if (gamepad1.y) {
                liftSystem.lift.moveElevatorToLevel(targetLevel, 2000);
                //while (gamepad2.x) { }

    /*            telemetry.addData("lifting to level 0", 0);
                telemetry.update();
                rSpindle.setTargetPosition(LEVEL_0);
                lSpindle.setTargetPosition(LEVEL_0);
                telemetry.addData("L Target:", lSpindle.getTargetPosition());
                telemetry.addData("R Target:", rSpindle.getTargetPosition());
                rSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.update();
                //while (gamepad2.x){};

                rSpindle.setPower(.99);
                lSpindle.setPower(.99);
                while(lSpindle.isBusy()){
                    telemetry.addData("lifting L:", lSpindle.getCurrentPosition());
                    telemetry.addData("lifting R:", rSpindle.getCurrentPosition());
                    telemetry.update();
                    teamUtil.log("lSpindle:"+lSpindle.getCurrentPosition());
               }
    */
            }
            if (gamepad2.dpad_left) {
                targetLevel--;
                teamUtil.sleep(500);
            }
            if (gamepad2.dpad_right) {
                targetLevel++;
                teamUtil.sleep(500);
            }
            if (gamepad2.a) {
                liftSystem.prepareToGrabNoWait(7000);
            }
            if (gamepad2.b) {
                liftSystem.grabAndStowNoWait("wide", 7000);
            }
            if (gamepad2.x) {
                liftSystem.hoverOverFoundationNoWait(targetLevel, Grabber.GrabberRotation.INSIDE, 7000);
            }
            if (gamepad2.y) {
                liftSystem.lift.moveElevatorToBottomNoWait();
            }
            if (gamepad2.left_bumper) {
                liftSystem.grab("wide",5000);
                //while (gamepad2.b) { }
            }
            if (gamepad2.dpad_up) {
                liftSystem.grabber.openGrabber();
            }
            if (gamepad2.left_bumper) {
                liftSystem.grab("wide",5000);
                //while (gamepad2.b) { }
            }
            if (gamepad2.dpad_down) {
                liftSystem.putAwayLiftSystemNoWait(8000);
            }


            //super cereal disclaimer. ts not sponsered its just amazing. or mabye im lieing. mabye this is straight our part of a plan for world domination. "that sounds rather unrealistic" you say with a gulible expression on your face. now let me ask you. what about knees. "knees?" you say with suprise. yes knees. what about them. they are funky looking and all bendy. but you know the ld saying. where there are knees there are bees
            liftSystem.lift.liftTelemetry();
            telemetry.addData("level:", targetLevel);
            telemetry.update();
        }
    }
}
