
package org.firstinspires.ftc.teamcode.TestCode.CoachCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "roboticArm")
public class roboticArm extends LinearOpMode {

    DcMotorEx baseMotor, elbowMotor;
    Servo wristServo, grabberServo;
    private DigitalChannel baseLimit, elbowLimit;

    public final double GRABBER_GRAB = 0.84; //
    public final double GRABBER_OPEN = 0.84; //
    public final double GRABBER_READY = 0.84; //

    public final double WRIST_STRAIGHT = 0.84; //Servo setting for a straight wrist
    public final double WRIST_DEGREES = 0.84; // Number of degrees the servo will rotate between position 0 and 1


    public void initialize() {
        baseMotor = hardwareMap.get(DcMotorEx.class,"bRightMotor");
        elbowMotor = hardwareMap.get(DcMotorEx.class,"elbowMotor");
        wristServo = hardwareMap.servo.get("wristServo");
        grabberServo = hardwareMap.servo.get("grabberServo");
        baseLimit = hardwareMap.get(DigitalChannel.class, "baseLimit");
        elbowLimit = hardwareMap.get(DigitalChannel.class, "elbowLimit");

        //baseMotor.setDirection(DcMotorSimple.Direction.REVERSE); // positive leans forward
        //elbowMotor.setDirection(DcMotorSimple.Direction.REVERSE); // postive extends

        //baseMotor.setVelocityPIDFCoefficients(1.5, 0.15,0,14.9); // these numbers were from a drive test...not calibrated for this arm!
        //elbowMotor.setVelocityPIDFCoefficients(1.5, 0.15,0,14.9); // these numbers were from a drive test...not calibrated for this arm!

        baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        baseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        telemetry.addData("Waiting to Start:", 0);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) {
                baseMotor.setVelocity(1500);
            } else if (gamepad1.a)
                baseMotor.setVelocity(-1500);
            else {
                baseMotor.setVelocity(0);
            }
            if (gamepad1.dpad_up) {
                elbowMotor.setVelocity(1500);
            } else if (gamepad1.dpad_down)
                elbowMotor.setVelocity(-1500);
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
            telemetry.addLine("Limits: wrist/grabber: "+baseLimit.getState()+"/"+elbowLimit.getState());
            telemetry.update();

        }
    }

}



