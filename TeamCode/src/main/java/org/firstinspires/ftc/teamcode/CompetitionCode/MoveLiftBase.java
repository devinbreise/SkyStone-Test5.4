package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;

@TeleOp(name = "MoveLiftBase")
public class MoveLiftBase extends OpMode {

    private DcMotor liftBase;

    public void init() {
        liftBase = hardwareMap.dcMotor.get("liftBase");
        liftBase.setDirection(DcMotorSimple.Direction.REVERSE);
        liftBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop() {
        if (gamepad1.y) {
            liftBase.setPower(0.5);
        } else if (gamepad1.a) {
            liftBase.setPower(-0.5);
        } else {
            liftBase.setPower(0);
        }
    }
}
