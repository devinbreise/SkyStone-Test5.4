package org.firstinspires.ftc.teamcode.TestCode.CoachCode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.basicLibs.DistanceSensors;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "CoachDistanceSensorTest")
public class coachDistanceSensor extends OpMode {
    Rev2mDistanceSensor frontLeftDistance, frontRightDistance, leftDistanceSensor, rightDistanceSensor, backDistanceSensor;

    public void init() {
        teamUtil.initPerf();

        frontLeftDistance = hardwareMap.get(Rev2mDistanceSensor.class, "frontLeftDistance");
        frontRightDistance = hardwareMap.get(Rev2mDistanceSensor.class, "frontRightDistance");
        leftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistance");
        rightDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightDistance");
        backDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "backDistance");
    }

    @Override
    public void loop() {
        telemetry.addData("frontLeftDistance", (int)frontLeftDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("frontRightDistance", (int)frontRightDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("leftDistance", (int)leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("rightDistance", (int)rightDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("backDistance", (int)backDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
        teamUtil.trackPerf();
    }
}
