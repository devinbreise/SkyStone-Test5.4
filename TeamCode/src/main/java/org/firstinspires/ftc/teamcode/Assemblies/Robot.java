package org.firstinspires.ftc.teamcode.Assemblies;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.HardwareMap;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

// A class to encapsulate the entire robot.
// This class is designed to be used ONLY in a linearOpMode (for Auto OR Teleop)
public class Robot {
    public static final double DISTANCE_TO_BLOCK = 3.2;
    public static final int MIN_DISTANCE_FOR_AUTO_DROPOFF = 6;
    public final double DISTANCE_TO_FOUNDATION = 2;
    public static final double AUTOINTAKE_POWER = 0.33;
    public static final double AUTOINTAKE_SIDEWAYS_POWER = 0.33;

    public LiftSystem liftSystem;
    public RobotDrive drive;
    public Latch latch;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    boolean timedOut = false;

    private final int MIN_DISTANCE_FOR_AUTO_PICKUP = 10;
    private final int MAX_DISTANCE_FOR_AUTO_DROPOFF = 10;

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
        liftSystem.initLiftSystem();
        drive.initImu();
        drive.initDriveMotors();
        drive.initSensors();
        drive.resetHeading();
        latch.initLatch();
        liftSystem.resetSpindles();
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

        // In case we were still getting the lift system ready to grab
        while (liftSystem.state!= LiftSystem.LiftSystemState.IDLE) {
            teamUtil.sleep(100);
        }
        liftSystem.grabAndStow("wide", timeOutTime - System.currentTimeMillis());

    }
    public void autoDropOff(long timeOut){
        long timeOutTime= System.currentTimeMillis()+timeOut;

        // Rotate is not that accurate at the moment, so we are relying on the driver


        // Get the lift system ready to grab if it isn't already...



        // determine which side we are lined up on

        // line up using the front left sensor
        if (drive.frontLeftDistance.getDistance()< MAX_DISTANCE_FOR_AUTO_DROPOFF && drive.frontLeftDistance.getDistance() > MIN_DISTANCE_FOR_AUTO_DROPOFF) {
            teamUtil.log("autointake -- see something in front left!");
            liftSystem.hoverOverFoundationNoWait(0, Grabber.GrabberRotation.INSIDE, 8500);
            teamUtil.sleep(5000);
            drive.moveToDistance(drive.frontLeftDistance, DISTANCE_TO_FOUNDATION, AUTOINTAKE_POWER, 5000);

            while((drive.frontLeftDistance.getDistance()< MAX_DISTANCE_FOR_AUTO_DROPOFF) && teamUtil.keepGoing(timeOutTime) && teamUtil.theOpMode.opModeIsActive()) {
                drive.driveLeft(AUTOINTAKE_POWER);
            }
            drive.moveInchesLeft(AUTOINTAKE_SIDEWAYS_POWER, 1, 8000);
            //drive.moveInchesLeft(AUTOINTAKE_SIDEWAYS_POWER, 5, timeOutTime - System.currentTimeMillis());

            liftSystem.drop();
            // line up using the front right sensor
        }

        // In case we were still getting the lift system ready to grab


    }
}

