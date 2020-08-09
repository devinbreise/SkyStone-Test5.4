
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

    public class Point {
        public double x,y;
        public Point(double X, double Y){
            x = X;
            y = Y;
        }
    }

    public class Angles {
        public double base,elbow,wrist;
        public Angles(double b, double e, double w){
            base = b;
            elbow = e;
            wrist = w;
        }
    }

    public final double GRABBER_GRAB = 0.15; //
    public final double GRABBER_OPEN = 0.84; //
    public final double GRABBER_READY = 0.6; //

    public final double WRIST_STRAIGHT = 0.46; //Servo setting for a straight wrist
    public final double WRIST_DEGREES = 270; // Number of degrees the servo will rotate between position 0 and 1

    public final double BASE_LENGTH = 38.5; //length of base arm joint to joint in cms
    public final double UPPER_LENGTH = 31; //length of forearm arm joint to joint in cms
    public final double ELBOW_DEGREES = 0.0131; // Number of degrees per encoder click
    public final double BASE_DEGREES = 0.0134; // Number of degrees per encoder click
    public final double ELBOW_LIMIT_ANGLE = 15; // Number of degrees between elbow limit base arm
    public final double BASE_LIMIT_ANGLE = 140; // Number of degrees between base limit and forward horizontal
    public final double ELBOW_MAX_VELOCITY = 2500; //  under load
    public final double BASE_MAX_VELOCITY = 2600;  //  under load


    public void initialize() {
        baseMotor = hardwareMap.get(DcMotorEx.class,"baseMotor");
        elbowMotor = hardwareMap.get(DcMotorEx.class,"elbowMotor");
        wristServo = hardwareMap.servo.get("wristServo");
        grabberServo = hardwareMap.servo.get("grabberServo");
        baseLimit = hardwareMap.get(DigitalChannel.class, "baseLimit");
        elbowLimit = hardwareMap.get(DigitalChannel.class, "elbowLimit");

        baseMotor.setDirection(DcMotorSimple.Direction.REVERSE); // positive leans forward
        elbowMotor.setDirection(DcMotorSimple.Direction.REVERSE); // postive extends

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
        baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Calibrate and reset forearm
        if (!elbowLimit.getState()) {  // if the limit switch is already triggered...
            // extend elbow until limit switch is no longer triggered, then go a bit more
            elbowMotor.setVelocity(1000);
            while (!elbowLimit.getState()) ;
            teamUtil.pause(500);
            elbowMotor.setVelocity(0);
        }
        // retract forearm until it triggers the limit switch
        elbowMotor.setVelocity(-1000);
        while (elbowLimit.getState()) ;
        elbowMotor.setVelocity(0);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    Point getWristLocation() {
        Point wristLocation = new Point(0,0);
        double B1 = getCurrentBaseAngle();
        double E = getCurrentElbowAngle();

        // compute the length of the missing side of the triangle formed by the arms
        double c = Math.sqrt(BASE_LENGTH*BASE_LENGTH + UPPER_LENGTH*UPPER_LENGTH - 2*BASE_LENGTH*UPPER_LENGTH * Math.cos(Math.toRadians(E)));

        // compute the angle between this 3rd side and the base arm
        double B = Math.asin(Math.sin(Math.toRadians(E)*BASE_LENGTH/c)); // B is in radians!

        // compute the angle between the 3rd side and horizontal
        double B2 = B1-Math.toDegrees(B);
        //teamUtil.log("wrist:"+c+"/"+B+"/"+B2);
        // Find the X and Y using a right triangle with  the "3rd side" as the hypotenuse
        wristLocation.x = Math.cos(Math.toRadians(B2)) * c;
        wristLocation.y = Math.sin(Math.toRadians(B2)) * c;
        return wristLocation;
    }

    Point getWristLocationOLD() {
        Point elbowLocation = new Point(0,0);
        Point wristLocation = new Point(0,0);
        double B = getCurrentBaseAngle();
        double E = getCurrentElbowAngle();
        if (B > 90) {
            elbowLocation.x = Math.sin(Math.toRadians(B-90)) * BASE_LENGTH * -1;
            elbowLocation.y = Math.cos(Math.toRadians(B-90)) * BASE_LENGTH;
        } else  {
            elbowLocation.x = Math.cos(Math.toRadians(B)) * BASE_LENGTH;
            elbowLocation.y = Math.sin(Math.toRadians(B)) * BASE_LENGTH;
        }
        if (E > 90) {
            wristLocation.x = Math.sin(Math.toRadians(B-90)) * UPPER_LENGTH * -1 + elbowLocation.x ;
            wristLocation.y = Math.cos(Math.toRadians(B-90)) * UPPER_LENGTH + elbowLocation.y;
        } else  {
            wristLocation.x = Math.cos(Math.toRadians(B)) * UPPER_LENGTH + elbowLocation.x;
            wristLocation.y = Math.sin(Math.toRadians(B)) * UPPER_LENGTH + elbowLocation.y;
        }
        return wristLocation;
    }

    Angles computeAngles(double x, double y) {
        double bl, al, ra, rl, a0, a1,a2,a3;

        bl = BASE_LENGTH; // length of arm base
        al = UPPER_LENGTH; // length of forearm
        if (x > 0)
            ra = Math.atan(y/x);  // angle between ground and wrist joint position in radians
        else
            ra = Math.atan(y/-x)+ Math.PI/2;  // angle between ground and wrist joint position in radians
        rl = Math.sqrt(x*x+y*y); // distance between arm base pivot point and wrist joint
        a0 = Math.acos((rl*rl+bl*bl-al*al)/(2.0*bl*rl));
        a1 = ra+a0; // angle between ground and front of arm base in radians
        a2 = Math.acos((al*al+bl*bl-rl*rl)/(2.0*bl*al)); // angle between arm base and upper arm in radians
        a3 = 2.0*Math.PI - ra - a0 - a2; // angle between upper arm and grabber assuming grabber level with ground in radians
        return new Angles(Math.toDegrees(a1), Math.toDegrees(a2), Math.toDegrees(a3));
        /*
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
*/
    }

    // returns the current angle between the base arm segment and horizontal (in front)
    double getCurrentBaseAngle() {
        return (BASE_LIMIT_ANGLE - BASE_DEGREES * baseMotor.getCurrentPosition());
    }
    // returns the current angle between the base arm segment and forearm segment (0 is folded in front)
    double getCurrentElbowAngle() {
        return ELBOW_LIMIT_ANGLE + ELBOW_DEGREES * elbowMotor.getCurrentPosition();
    }

    void moveArm() {
        // Get the desired movement vector
        float stickX = gamepad1.right_stick_x;
        float stickY = -gamepad1.right_stick_y;

        // Provide a generous dead zone to avoid unwanted movement and scale the rest
        if (Math.abs(stickX) < 0.2)
            stickX = 0;
        else
            if (stickX > 0 )
                stickX = (float) ((stickX - 0.2)* 1.25);
            else
                stickX = (float) ((stickX + 0.2)* 1.25);
        if (Math.abs(stickY) < 0.2)
            stickY = 0;
        else
            if (stickY > 0 )
                stickY = (float) ((stickY - 0.2)* 1.25);
            else
                stickY = (float) ((stickY + 0.2)* 1.25);

        if ((Math.abs(stickX) == 0) && (Math.abs(stickY) == 0))    { // stop the arm
            telemetry.addData("eV:", 0);
            telemetry.addData("bV:", 0);
            baseMotor.setVelocity(0);
            elbowMotor.setVelocity(0);
        } else { // get/keep the arm moving in the right direction
            // Given the wrist joint's current position in space,
            // determine the ratio of base to elbow movement and the signs on each
            double currentBaseAngle = getCurrentBaseAngle();
            double currentElbowAngle = getCurrentElbowAngle();

            // Determine the target x and y position for the wrist joint based on current position and joy stick input
            Point wristLocation = getWristLocation();
            telemetry.addLine("Wrist:"+wristLocation.x+","+ wristLocation.y);
            teamUtil.log("Wrist:"+wristLocation.x+","+ wristLocation.y);

            double targetX = wristLocation.x + stickX;
            double targetY = wristLocation.y + stickY;
            teamUtil.log("Target:"+targetX+","+ targetY);

            // Determine the target angles for the two joints given the target position
            Angles targetAngles = computeAngles(targetX, targetY);
            teamUtil.log("targetAngles:"+targetAngles.base+"/"+targetAngles.elbow+"/"+targetAngles.wrist);

            // Determine how many encoder clicks each motor needs to change and in what direction
            double elbowDiff = (targetAngles.elbow-currentElbowAngle) * ELBOW_DEGREES;
            double baseDiff = (targetAngles.base-currentBaseAngle) * BASE_DEGREES;
            teamUtil.log("Diffs:"+baseDiff+","+ elbowDiff);

            // Set the motors to their new velocities to move straight towards the target wrist position
            double elbowVelocity, baseVelocity;
            double velocityScale = Math.sqrt(stickX*stickX + stickY*stickY);
            if (Math.abs(elbowDiff) > Math.abs(baseDiff)) {
                elbowVelocity = velocityScale * ELBOW_MAX_VELOCITY;
                baseVelocity = (baseDiff / elbowDiff) * velocityScale * BASE_MAX_VELOCITY;
            } else {
                baseVelocity = velocityScale * BASE_MAX_VELOCITY;
                elbowVelocity = (elbowDiff / baseDiff) * velocityScale * ELBOW_MAX_VELOCITY;
            }

            // TODO: Need code here to impose limits on movement in both directions
            teamUtil.log("Vs:"+baseVelocity+","+ elbowVelocity);

            telemetry.addData("eV:", elbowVelocity);
            telemetry.addData("bV:", baseVelocity);
            //elbowMotor.setVelocity(elbowVelocity);
            //baseMotor.setVelocity(baseVelocity);
        }
    }

    public void runOpMode() {
        teamUtil.init(this);

        initialize();
        calibrateAndReset();

        telemetry.addData("Waiting to Start:", 0);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double maxVelocityE = 0;
            double maxVelocityB = 0;
            if (gamepad1.right_trigger > .8 && gamepad1.left_trigger > .8) {
                if (gamepad1.y) {
                    baseMotor.setVelocity(1000);
                } else if (gamepad1.a)
                    baseMotor.setVelocity(-1000);
                else if (gamepad1.dpad_right) {
                    baseMotor.setVelocity(10000);
                    if (baseMotor.getVelocity()> maxVelocityB)
                        maxVelocityB = baseMotor.getVelocity();
                    teamUtil.log("base: "+ maxVelocityB);
                } else {
                    baseMotor.setVelocity(0);
                }
                if (gamepad1.dpad_up) {
                    elbowMotor.setVelocity(1000);
                } else if (gamepad1.dpad_down)
                    elbowMotor.setVelocity(-1000);
                else if (gamepad1.dpad_left) {
                    elbowMotor.setVelocity(10000);
                    if (elbowMotor.getVelocity()> maxVelocityE)
                        maxVelocityE = elbowMotor.getVelocity();
                    teamUtil.log("elbow: "+ maxVelocityE);
                } else  {
                    elbowMotor.setVelocity(0);
                }

            } else {
                moveArm();
            }

            if (gamepad1.right_bumper) {
                grabberServo.setPosition(GRABBER_GRAB);
            } else if (gamepad1.left_bumper) {
                grabberServo.setPosition(GRABBER_OPEN);
            }
            Point wristLocation = getWristLocation();
            telemetry.addLine("base/elbow encoder: "+baseMotor.getCurrentPosition()+"/"+elbowMotor.getCurrentPosition());
            telemetry.addLine("base/elbow degrees: "+getCurrentBaseAngle()+"/"+getCurrentElbowAngle());
            telemetry.addLine("wrist location: "+wristLocation.x + "," + wristLocation.y);
            telemetry.addLine("wrist/grabber: "+wristServo.getPosition()+"/"+grabberServo.getPosition());
            telemetry.addLine("Limits: base/elbow: "+baseLimit.getState()+"/"+elbowLimit.getState());
            telemetry.update();

        }
    }

}



