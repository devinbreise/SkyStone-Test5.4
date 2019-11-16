
package org.firstinspires.ftc.teamcode.TestCode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "demoBot")
@Disabled
public class demoBot extends OpMode {
    public static final double ROTATE_GRABBER_INCREMENT = 0.1;


    //two servos for closing, one for rotating on top

    public static double MAX_POWER = 1;



    private DcMotor flingerOne, flingerTwo;
    private DcMotor leftMotor, rightMotor;

    public void init() {

        flingerOne = hardwareMap.get(DcMotor.class, "flingerOne");
        flingerTwo = hardwareMap.get(DcMotor.class, "flingerTwo");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        flingerTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        flingerOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flingerTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flingerOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flingerTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop() {
        double power = 0.5;
        power = gamepad1.left_trigger;

        rightMotor.setPower(gamepad1.left_stick_y);
        leftMotor.setPower(gamepad1.right_stick_y);
        flingerOne.setPower(power);
        flingerTwo.setPower(power*0.75);

        telemetry.update();

    }
}



