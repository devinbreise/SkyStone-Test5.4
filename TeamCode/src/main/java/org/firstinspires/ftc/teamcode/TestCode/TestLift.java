package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Lift;
import org.firstinspires.ftc.teamcode.Assemblies.LiftSystem;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "TestLift")
public class TestLift extends OpMode {

    private double LIFT_BASE_POWER = 1;
    private int targetLevel = 0;

    private Lift lift;
    private LiftSystem liftSystem;



    public void init() {

        lift = new Lift(hardwareMap, telemetry);
        liftSystem = new LiftSystem(hardwareMap, telemetry);
        lift.initLift();
        liftSystem.initLiftSystem();
        //lift.tensionLiftString();

    }

    public void loop() {

        // manual limbo
        if (gamepad1.b) {
           lift.upPosition(0.99, 7000);
            //lift.liftBaseUp();
        } else if (gamepad1.a) {
            lift.downPosition(0.3, 7000);
           //lift.liftBaseDown();
        } //else lift.shutDownLiftBase();

        if (gamepad1.dpad_down) {
            lift.decreaseLiftBasePower();
        }
        if (gamepad1.dpad_up) {
            lift.increaseLiftBasePower();
        }
        if (gamepad1.dpad_left) {
            targetLevel--;
            teamUtil.sleep(500);
        }
        if (gamepad1.dpad_right) {
            targetLevel++;
            teamUtil.sleep(500);        }

        // manual elevator control
        if (gamepad1.y) {
            lift.tensionLiftString();
            //while (gamepad2.a) {};
        }
        if (gamepad1.x) {
            lift.goToLevel(0, 2000);
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
        if(gamepad2.right_bumper){
            liftSystem.grabAndStowNoWait("wide", 7000);
        }
        if(gamepad2.left_bumper){
            liftSystem.prepareToGrabNoWait(7000);
        }
        if(gamepad2.dpad_up){
            liftSystem.hoverOverFoundationNoWait(0, Grabber.GrabberRotation.OUTSIDE, 7000);
        }
        if (gamepad2.y) {
            lift.goToLevel(targetLevel, 5000);
            //while (gamepad2.y) { }
        }
        if (gamepad2.b) {
            lift.goToBottom();
            //while (gamepad2.b) { }
        }

//super cereal disclaimer. ts not sponsered its just amazing. or mabye im lieing. mabye this is straight our part of a plan for world domination. "that sounds rather unrealistic" you say with a gulible expression on your face. now let me ask you. what about knees. "knees?" you say with suprise. yes knees. what about them. they are funky looking and all bendy. but you know the ld saying. where there are knees there are bees
        lift.liftTelemetry();
//        telemetry.addData("level:", targetLevel);
//        telemetry.addData("EncoderPosition:", lift.getBasePosition());
        telemetry.update();
    }
}
