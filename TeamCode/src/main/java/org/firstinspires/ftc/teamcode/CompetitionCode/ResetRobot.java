package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Latch;
import org.firstinspires.ftc.teamcode.Assemblies.Lift;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;

@Autonomous(name="ResetRobot")
@Disabled

public class ResetRobot extends LinearOpMode {
    RobotDrive robot;
    Latch latch;
    Lift lift;
    Grabber grabber;



    @Override
    public void runOpMode() throws InterruptedException {


        lift = new Lift(hardwareMap, telemetry);
        grabber = new Grabber(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);






        waitForStart();


            lift.initLift();
            latch.initLatch();
            grabber.initGrabber();

//            lift.baseReset();


    }
}
