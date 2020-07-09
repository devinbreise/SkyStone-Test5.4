
package org.firstinspires.ftc.teamcode.TestCode.CoachCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "roboticArm")
public class roboticArm extends LinearOpMode {

    DcMotorEx baseMotor, elbowMotor;
    Servo wristServo, grabberServo;
    private DigitalChannel baseLimit, elbowLimit;

    public final double GRABBER_GRAB = 0.15; //
    public final double GRABBER_OPEN = 0.84; //
    public final double GRABBER_READY = 0.6; //

    public final double WRIST_STRAIGHT = 0.46; //Servo setting for a straight wrist
    public final double WRIST_DEGREES = 270; // Number of degrees the servo will rotate between position 0 and 1

    public final double ELBOW_DEGREES = 0.1557; // Number of degrees per encoder click
    public final double BASE_DEGREES = 0.011652; // Number of degrees per encoder click
    public final double ELBOW_LIMIT_ANGLE = 1; // TODO Number of degrees between elbow limit and straight
    public final double BASE_LIMIT_ANGLE = 1; // TODO Number of degrees between base limit and vertical


    public void initialize() {
        baseMotor = hardwareMap.get(DcMotorEx.class,"baseMotor");
        elbowMotor = hardwareMap.get(DcMotorEx.class,"elbowMotor");
        wristServo = hardwareMap.servo.get("wristServo");
        grabberServo = hardwareMap.servo.get("grabberServo");
        baseLimit = hardwareMap.get(DigitalChannel.class, "baseLimit");
        elbowLimit = hardwareMap.get(DigitalChannel.class, "elbowLimit");

        baseMotor.setDirection(DcMotorSimple.Direction.REVERSE); // positive leans forward
        //elbowMotor.setDirection(DcMotorSimple.Direction.REVERSE); // postive extends

        //baseMotor.setVelocityPIDFCoefficients(1.5, 0.15,0,14.9); // these numbers were from a drive test...not calibrated for this arm!
        //elbowMotor.setVelocityPIDFCoefficients(1.5, 0.15,0,14.9); // these numbers were from a drive test...not calibrated for this arm!

        baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        baseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void calibrateAndReset() {
        wristServo.setPosition(WRIST_STRAIGHT);
        grabberServo.setPosition(GRABBER_READY);

        // Get the elbow motor to hold the forearm in place
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setTargetPosition(0);
        elbowMotor.setVelocity(1500);

        // Calibrate and reset Arm Base
        if (!baseLimit.getState()) { // if the limit switch is already triggered...
            // move arm base forward until limit switch is no longer triggered, then go a bit more
            baseMotor.setVelocity(1000);
            while (!baseLimit.getState()) ;
            teamUtil.pause(500);
            baseMotor.setVelocity(0);
        }
        // lean base motor back until it triggers the limit switch
        baseMotor.setVelocity(-1000);
        while (baseLimit.getState()) ;
        baseMotor.setVelocity(0);
        baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        baseMotor.setTargetPosition(0);
        baseMotor.setVelocity(1500);

        // Calibrate and reset forearm
        if (!elbowLimit.getState()) {  // if the limit switch is already triggered...
            // extend elbow until limit switch is no longer triggered, then go a bit more
            elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbowMotor.setVelocity(1000);
            while (!elbowLimit.getState()) ;
            teamUtil.pause(500);
            elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setTargetPosition(0);
            elbowMotor.setVelocity(1500);
        }
        // retract forearm until it triggers the limit switch
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setVelocity(1000);
        while (elbowLimit.getState()) {
            elbowMotor.setTargetPosition(elbowMotor.getTargetPosition() - 10);
            teamUtil.pause(100);
        }
        elbowMotor.setVelocity(0);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setTargetPosition(0);
        elbowMotor.setVelocity(1000);
    }


    void computeAngles() {
        double x,y,bl, al, ra, rl, a0, a1,a2,a3;
        x = 4.0;
        y = 4.0;
        bl = 4;
        al = 2.5;
        ra = Math.atan(y/x);
        rl = Math.sqrt(x*x+y*y);
        a0 = Math.acos((rl*rl+bl*bl-al*al)/(2.0*bl*rl));
        a1 = ra+a0; // angle between ground and front of arm base
        a2 = Math.acos((al*al+bl*bl-rl*rl)/(2.0*bl*al)); // angle between arm base and upper arm
        a3 = 2.0*Math.PI - ra - a0 - a2; // angle between upper arm and grabber assuming grabber level with ground
        System.out.println("x="+x);
        System.out.println("y="+y);
        System.out.println("ra="+Math.toDegrees(ra));
        System.out.println("bl="+bl);
        System.out.println("al="+al);
        System.out.println("rl="+rl);
        System.out.println("a0="+Math.toDegrees(a0));
        System.out.println("a1="+Math.toDegrees(a1));
        System.out.println("a2="+Math.toDegrees(a2));
        System.out.println("a3="+Math.toDegrees(a3));

    }
    public void runOpMode() {
        teamUtil.init(this);

        initialize();
        calibrateAndReset();

        telemetry.addData("Waiting to Start:", 0);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) {
                baseMotor.setVelocity(1000);
            } else if (gamepad1.a)
                baseMotor.setVelocity(-1000);
            else {
                baseMotor.setVelocity(0);
            }
            if (gamepad1.dpad_up) {
                elbowMotor.setVelocity(500);
            } else if (gamepad1.dpad_down)
                elbowMotor.setVelocity(-500);
            else {
                elbowMotor.setVelocity(0);
            }
            if (gamepad1.right_bumper) {
                grabberServo.setPosition(GRABBER_GRAB);
            } else if (gamepad1.left_bumper) {
                grabberServo.setPosition(GRABBER_OPEN);
            }

            telemetry.addLine("base/elbow: "+baseMotor.getCurrentPosition()+"/"+elbowMotor.getCurrentPosition());
            telemetry.addLine("wrist/grabber: "+wristServo.getPosition()+"/"+grabberServo.getPosition());
            telemetry.addLine("Limits: base/elbow: "+baseLimit.getState()+"/"+elbowLimit.getState());
            telemetry.update();

        }
    }

}



