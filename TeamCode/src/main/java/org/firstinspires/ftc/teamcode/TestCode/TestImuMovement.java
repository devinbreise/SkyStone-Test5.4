package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name = "TestImuMovement")
public class TestImuMovement extends LinearOpMode {


    RobotDrive robot;



    @Override
    public void runOpMode() throws InterruptedException {
        teamUtil.theOpMode = this;
        robot = new RobotDrive(hardwareMap, telemetry);

        robot.initDriveMotors();
        robot.initImu();
        telemetry.addData("heading:", robot.getHeading());
        telemetry.update();


        while(!opModeIsActive()){
            robot.resetHeading();
            telemetry.addData("heading:", robot.getHeading());
            telemetry.update();
        }

        teamUtil.log("currentHeading: " + robot.getHeading());

//        robot.universalJoystick(0, 1, 0, 0.5,  225);


            sleep(8000);



//                teamUtil.log("heading: " + robot.getHeading());




    }
}
