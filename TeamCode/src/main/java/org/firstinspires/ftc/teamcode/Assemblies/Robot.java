package org.firstinspires.ftc.teamcode.Assemblies;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.HardwareMap;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

// A class to encapsulate the entire robot.
// This class is designed to be used ONLY in a linearOpMode (for Auto OR Teleop)
public class Robot {
    public LiftSystem liftSystem;
    public RobotDrive drive;
    public Latch latch;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    boolean timedOut = false;

    int MIN_DISTANCE_FOR_AUTO_PICKUP = 15;

    public Robot(LinearOpMode opMode){
        teamUtil.log ("Constructing Robot");
        // stash some context for later
        teamUtil.theOpMode = opMode;
        telemetry = opMode.telemetry;
        hardwareMap = opMode.hardwareMap;

        teamUtil.log ("Constructing Assemblies");
        liftSystem = new LiftSystem(hardwareMap, telemetry);
        drive = new RobotDrive(hardwareMap, telemetry);
        latch = new Latch(hardwareMap, telemetry);
        teamUtil.log ("Constructing Assemblies - Finished");
        teamUtil.log ("Constructed Robot - Finished");
    }

    // Call this before first use!
    public void init(){
        teamUtil.log ("Initializing Robot");
        drive.initImu();
        drive.initDriveMotors();
        drive.initDistanceSensors();
        drive.resetHeading();
        liftSystem.initLiftSystem();
        latch.initLatch();
        teamUtil.log ("Initializing Robot - Finished");
    }

    ////////////////////////////////////////////////////
    // Automagically align on a stone, pick it up, and stow it.
    // stop if we don't finish maneuvering within timeOut msecs
    public void autoIntake(boolean isBlue, long timeOut){
        long timeOutTime= System.currentTimeMillis()+timeOut;

        // Rotate is not that accurate at the moment, so we are relying on the driver
        if(isBlue) {
            //drive.imuRotateToAngle(180);
        }
        else {
            //drive.imuRotateToAngle(0);
        }

        // Get the lift system ready to grab if it isn't already...
        liftSystem.prepareToGrabNoWait(7000);

        // determine which side we are lined up on

        // line up using the front left sensor
        if (drive.frontLeftDistance.getDistance()< MIN_DISTANCE_FOR_AUTO_PICKUP) {
            drive.moveToDistance(drive.frontLeftDistance, 3, 0.25, 5000);
            while((drive.frontLeftDistance.getDistance()<10) && teamUtil.keepGoing(timeOutTime)) {
                drive.driveLeft(0.3);
            }
            drive.moveInchesLeft(.3, 2, timeOutTime - System.currentTimeMillis());

            // line up using the front right sensor
        } else if (drive.frontRightDistance.getDistance()< MIN_DISTANCE_FOR_AUTO_PICKUP) {
            drive.moveToDistance(drive.frontRightDistance, 3, 0.3, 5000);
            while ((drive.frontRightDistance.getDistance() < 10) && teamUtil.keepGoing(timeOutTime)) {
                drive.driveRight(0.3);
            }
            drive.moveInchesRight(.3, 2, timeOutTime - System.currentTimeMillis());
        }

        // In case we were still getting the lift system ready to grab
        while (liftSystem.state!= LiftSystem.LiftSystemState.IDLE) {
            teamUtil.sleep(100);
        }
        liftSystem.grabAndStow("wide", timeOutTime - System.currentTimeMillis());

    }
}

