package org.firstinspires.ftc.teamcode.Assemblies;

        import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.HardwareMap;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
        import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

// A class to encapsulate the entire robot.
// This class is designed to be used ONLY in a linearOpMode (for Auto OR Teleop)
public class Robot {
    public static final double DISTANCE_TO_BLOCK = 1.125;
    public static final int MIN_DISTANCE_FOR_AUTO_DROPOFF = 6;
    public final double DISTANCE_TO_FOUNDATION = 2.5;
    public static final double AUTOINTAKE_POWER = 0.33;
    public static final double AUTOINTAKE_SIDEWAYS_POWER = 0.33;
    public boolean hasBeenInitialized = false;

    public LiftSystem liftSystem;
    public RobotDrive drive;
    public Latch latch;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    boolean timedOut = false;

    public final int MIN_DISTANCE_FOR_AUTO_PICKUP = 0;
    public final int MAX_DISTANCE_FOR_AUTO_DROPOFF = 6;

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
    public void init(boolean usingDistanceSensors){
            teamUtil.log ("Initializing Robot");
            liftSystem.initLiftSystem();
            drive.initImu();
            drive.initDriveMotors();
            if(usingDistanceSensors){
                drive.initSensors(false);
            }
            drive.resetHeading();
            latch.initLatch();
            teamUtil.log ("Initializing Robot - Finished");
    }

    ////////////////////////////////////////////////////
    // Automagically align on a stone, pick it up, and stow it.
    // stop if we don't finish maneuvering within timeOut msecs
    public void autoIntake(long timeOut){
        long timeOutTime= System.currentTimeMillis()+timeOut;

        // Rotate is not that accurate at the moment, so we are relying on the driver


        // Get the lift system ready to grab if it isn't already...
        liftSystem.prepareToGrabNoWait(7000);

        // determine which side we are lined up on

        // line up using the front left sensor


        if (drive.frontLeftDistance.getDistance()< MIN_DISTANCE_FOR_AUTO_PICKUP) {
            teamUtil.log("autointake -- see something in front left!");
            drive.moveToDistance(drive.frontLeftDistance, DISTANCE_TO_BLOCK, AUTOINTAKE_POWER, 5000);
            while((drive.frontLeftDistance.getDistance()<MIN_DISTANCE_FOR_AUTO_PICKUP) && teamUtil.keepGoing(timeOutTime) && teamUtil.theOpMode.opModeIsActive()) {
                drive.driveLeft(AUTOINTAKE_POWER);
            }
            drive.moveInchesLeft(AUTOINTAKE_SIDEWAYS_POWER, 2, timeOutTime - System.currentTimeMillis());

            // line up using the front right sensor
        } else if (drive.frontRightDistance.getDistance()< MIN_DISTANCE_FOR_AUTO_PICKUP) {
            teamUtil.log("autointake -- see something in front right!");
            drive.moveToDistance(drive.frontRightDistance, DISTANCE_TO_BLOCK, AUTOINTAKE_POWER, 5000);
            while ((drive.frontRightDistance.getDistance() < MIN_DISTANCE_FOR_AUTO_PICKUP) && teamUtil.keepGoing(timeOutTime) && teamUtil.theOpMode.opModeIsActive()) {
                drive.driveRight(AUTOINTAKE_POWER);
            }
            drive.moveInchesRight(AUTOINTAKE_SIDEWAYS_POWER, 2, timeOutTime - System.currentTimeMillis());
        }
        drive.moveToPickUpDistance(5000);
        teamUtil.sleep(375);
        drive.moveToPickUpDistance(2000);
        // In case we were still getting the lift system ready to grab
        while (liftSystem.state!= LiftSystem.LiftSystemState.IDLE) {
            teamUtil.sleep(100);
        }

        liftSystem.grabAndStowNoWait(7000); // return control to driver
        teamUtil.sleep(1000); // after a short pause to make sure we have picked up the stone

    }

    public void autoDropOffLeft(int level, long timeOut){
        long timeOutTime= System.currentTimeMillis()+timeOut;

        // Rotate is not that accurate at the moment, so we are relying on the driver


        // Get the lift system ready to grab if it isn't already...



        // determine which side we are lined up on

        // line up using the front left sensor
            teamUtil.log("autointake -- see something in front left!");
            liftSystem.hoverOverFoundationNoWait(level, Grabber.GrabberRotation.INSIDE, 8500);
            teamUtil.sleep(2000);
            drive.moveToDistance(drive.frontLeftDistance, DISTANCE_TO_FOUNDATION, AUTOINTAKE_POWER, 5000);

            while((drive.frontLeftDistance.getDistance()< MAX_DISTANCE_FOR_AUTO_DROPOFF) && teamUtil.keepGoing(timeOutTime) && teamUtil.theOpMode.opModeIsActive()) {
                drive.driveLeft(AUTOINTAKE_POWER);
            }
            drive.moveInchesLeft(AUTOINTAKE_SIDEWAYS_POWER, 1, 8000);
            //drive.moveInchesLeft(AUTOINTAKE_SIDEWAYS_POWER, 5, timeOutTime - System.currentTimeMillis());

            // line up using the front right sensor
        }

        // In case we were still getting the lift system ready to grab



    public void autoDropOffRight(int level, long timeOut){
        long timeOutTime= System.currentTimeMillis()+timeOut;

        // Rotate is not that accurate at the moment, so we are relying on the driver


        // Get the lift system ready to grab if it isn't already...



        // determine which side we are lined up on

        // line up using the front left sensor
        teamUtil.log("autointake -- see something in front left!");
        liftSystem.hoverOverFoundationNoWait(level, Grabber.GrabberRotation.INSIDE, 8500);
        teamUtil.sleep(2000);
        drive.moveToDistance(drive.frontRightDistance, DISTANCE_TO_FOUNDATION, AUTOINTAKE_POWER, 5000);

        while((drive.frontRightDistance.getDistance()< MAX_DISTANCE_FOR_AUTO_DROPOFF) && teamUtil.keepGoing(timeOutTime) && teamUtil.theOpMode.opModeIsActive()) {
            drive.driveRight(AUTOINTAKE_POWER);
        }
        drive.moveInchesRight(AUTOINTAKE_SIDEWAYS_POWER, 1, 8000);
        //drive.moveInchesLeft(AUTOINTAKE_SIDEWAYS_POWER, 5, timeOutTime - System.currentTimeMillis());

        // line up using the front right sensor
    }

    public void foundationRed(){
        latch.latchDown();
        teamUtil.sleep(500);
        drive.moveInches(44, 0.7, 0.875, 1, 0.575 ,3273);
    }

    public void foundationBlue(){
        latch.latchDown();
        teamUtil.sleep(500);
        drive.moveInches(44, 0.975, 0.6, 0.675, 0.8 ,3273);
    }


    public void positionToFoundation(){
        liftSystem.hoverOverFoundationNoWait(0, Grabber.GrabberRotation.INSIDE, 5000);
        drive.closeToDistanceOr(drive.frontLeftDistance, drive.frontRightDistance, 4, 0.3, 4000);

        if((drive.frontRightDistance.getDistance() < MAX_DISTANCE_FOR_AUTO_DROPOFF) && (drive.frontLeftDistance.getDistance() > MAX_DISTANCE_FOR_AUTO_DROPOFF)){
            drive.moveToDistance(drive.frontRightDistance, DISTANCE_TO_FOUNDATION, AUTOINTAKE_POWER, 5000);

            while((drive.frontRightDistance.getDistance() < MAX_DISTANCE_FOR_AUTO_DROPOFF) && (drive.frontLeftDistance.getDistance() > MAX_DISTANCE_FOR_AUTO_DROPOFF) && teamUtil.keepGoing(6500 + System.currentTimeMillis()) && teamUtil.theOpMode.opModeIsActive()) {
                drive.driveRight(0.5);
                teamUtil.log("FrontLeftDistance: " + drive.frontLeftDistance.getDistance());
            }
            drive.moveInchesRight(0.6, 7.5, 4500);

        } else if((drive.frontRightDistance.getDistance() > MAX_DISTANCE_FOR_AUTO_DROPOFF) && (drive.frontLeftDistance.getDistance() < MAX_DISTANCE_FOR_AUTO_DROPOFF)){
            drive.moveToDistance(drive.frontLeftDistance, DISTANCE_TO_FOUNDATION, AUTOINTAKE_POWER, 5000);

            while((drive.frontRightDistance.getDistance() > MAX_DISTANCE_FOR_AUTO_DROPOFF) && (drive.frontLeftDistance.getDistance() < MAX_DISTANCE_FOR_AUTO_DROPOFF) && teamUtil.keepGoing(6500 + System.currentTimeMillis()) && teamUtil.theOpMode.opModeIsActive()) {
                drive.driveLeft(0.5);
                teamUtil.log("FrontLeftDistance: " + drive.frontLeftDistance.getDistance());
            }
            drive.moveInchesLeft(0.6, 11.5, 4500);
        } else {
            teamUtil.log("didn't see anything on distance sensors");
            return;
        }
    }


}


