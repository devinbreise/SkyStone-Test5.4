package org.firstinspires.ftc.teamcode.TestCode.CoachCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name="motorPIDTest")
@Disabled
public class motorPIDTest extends OpMode {

    private DcMotor theMotor;

    public void init() {
        theMotor = hardwareMap.get(DcMotor.class, "motor");
        telemetry.addData("Status", "Resetting Encoder");    //
        telemetry.update();

        theMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status",  "Starting at %7d", theMotor.getCurrentPosition());
        telemetry.update();

        theMotor.setTargetPosition(0); // 2 revolutions
        theMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void loop() {
        double power = 0;
        power = gamepad1.left_trigger;

        if (gamepad1.a) {
            theMotor.setTargetPosition((int)(teamUtil.NEVERREST40_ENCODER_CLICKS*2)); // 5 revolutions
        } else if (gamepad1.b){
            theMotor.setTargetPosition(0);
        }

        theMotor.setPower(gamepad1.left_stick_y);


        telemetry.addData("Target:",  theMotor.getTargetPosition());
        telemetry.addData("Position:",  theMotor.getCurrentPosition());
        telemetry.addData("Power:",  power);
        telemetry.update();

    }
}