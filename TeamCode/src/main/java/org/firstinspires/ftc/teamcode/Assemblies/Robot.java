package org.firstinspires.ftc.teamcode.Assemblies;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.HardwareMap;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
        import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

// A class to encapsulate the entire 
// This class is designed to be used ONLY in a linearOpMode (for Auto OR Teleop)
public class Robot {
    public static final double DISTANCE_TO_BLOCK = 1.125;
    public static final int MIN_DISTANCE_FOR_AUTO_DROPOFF = 6;
    public final double DISTANCE_TO_FOUNDATION = 2.5;
    public static final double AUTOINTAKE_POWER = 0.33;
    public static final double AUTOINTAKE_SIDEWAYS_POWER = 0.33;
    int path;
    public boolean hasBeenInitialized = false;

    public LiftSystem liftSystem;
    public RobotDrive drive;
    public Latch latch;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    SkystoneDetector detector;
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
    
    public void doubleSkystone(){

        boolean RED = (teamUtil.alliance == teamUtil.Alliance.RED);
        boolean useDistanceSensorsALot = false;
        double distance = 0;
        detector = new SkystoneDetector(telemetry, hardwareMap);
        detector.initDetector();
        detector.activateDetector();

        teamUtil.telemetry.addLine("Ready to Start");
        teamUtil.telemetry.update();

        // Start detecting but wait for start of match to move
        while (!teamUtil.theOpMode.opModeIsActive() && !teamUtil.theOpMode.isStopRequested()) {
            teamUtil.sleep(200);

            int detected = (RED ? detector.detectRed() : detector.detectBlue());
            if (detected > 0) {
                path = detected;
            }
        }

        detector.shutdownDector();

        if(teamUtil.theOpMode.isStopRequested()){
            return;
        }

        /////////////////////////////////////////////////////////
        // Move to the first Skystone and grab it
        liftSystem.prepareToGrabNoWait(4000);
        switch (path) {
            case 3 :
                // move straight forward
                break;
            case 2 :
                if (RED)
                    drive.moveInchesLeft(0.35, 7, 2300);
                else
                    drive.moveInchesRight(0.35, 7, 2300);

                //TODO: FIX FOR BLUE
                break;
            case 1 :
                if (RED)
                    drive.moveInchesLeft(0.35, 15, 2300);
                else
                    drive.moveInchesRight(0.35, 15, 2300);
                //TODO: FIX FOR BLUE
        }
        // TODO: will the strafes to the left/right above effect the location of the robot relative to the wall (and later the skybridge)?
        // TODO: If so, perhaps the rear distance sensor could come in handy here to normalize that distance before we move forward
        drive.newAccelerateInchesFB(1100, 32, 0, 3000);
        liftSystem.grabAndStowNoWait(4500);
        teamUtil.sleep(750);

        /////////////////////////////////////////////////////////
        // Deliver the first stone to the building side of the field
        drive.newAccelerateInchesFB(-2200, (RED ? 5 : 8/*TODO*/), 0, 3000);
        drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_BUILDING);
        switch (path) { // TODO: OR, we could go back to finding the tape line as we cross it and moving a set distance from there...
            case 3 : distance = (RED ? 44.5  :44.5/*TODO*/); break;
            case 2 : distance = (RED ? 52.5  :52.5/*TODO*/); break;// TODO RED + 8?
            case 1 : distance = (RED ? 60.5  :60.5/*TODO*/); break;// TODO RED + 8?
        }
        drive.newAccelerateInchesFB(2200,distance,RED ? 268: 88,5000);
        drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_FIELD);

        //If Blue, move forward a little 'cause long strafe will be towards center of field
        if(!RED){
            drive.newAccelerateInchesFB(2200, 4, 0, 2000);
        }
        //lift base up a teensy bit and drop off stone
        liftSystem.lift.slightlyMoveLiftBaseUp(1, 2000);
        liftSystem.grabber.slightlyOpenGrabber();
        teamUtil.sleep(750);

        /////////////////////////////////////////////////////////
        // drive back and position for the second stone
        liftSystem.lift.moveLiftBaseDownNoWait(0.5, 3000);

        //if it's path 1, we give up on double skystone ¯\_(ツ)_/¯
        if(path == 1){
            while (!drive.bottomColor.isOnTape()) {
                if (RED) {
                    drive.driveLeft(0.75);
                } else {
                    drive.driveRight(0.75);
                }
            }
            drive.stopMotors();
            return;
        }
        //if RED, move backward a little to avoid collision with skybridge
        if(RED){
            drive.newAccelerateInchesFB(-2200,3,0,5000);
        }

        drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_DEPOT);
        liftSystem.prepareToGrabNoWaitWithDelay(1000, 4500);
        switch (path) { // TODO: OR, we could go back to finding the tape line as we cross it and moving a set distance from there...
            case 3 :
            case 2 : distance = (RED ? 60  :60/*TODO*/); break;// TODO RED + 8?
//            case 1 : distance = (RED ? 60  :60/*TODO*/); break;// TODO Need to think about this case carefully!
        }
        drive.newAccelerateInchesFB(2200,distance, RED ? 89: 265.5,5000); // TODO: It would be nice to combine this movement with the close to distance...it might just work...
        switch (path) {
            case 3 :
            case 2 : distance = (RED ? 13.5  :13.5/*TODO*/); break; // TODO RED - 8?
//            case 1 : distance = (RED ? 10.5  :0/*TODO*/); break; // TODO Need to think about this case carefully!
        }
        drive.newMoveToDistance(drive.frontRightDistance, distance,1500, RED ? 89: 271 , true,4000);
        drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_FIELD);
        //strafe a tad if we're doing path 2 to line up to the stone
        if(path == 2){
            if(RED){
                drive.moveInchesLeft(0.35, 7, 2300);
            }else{
                drive.moveInchesRight(0.35, 7, 2300);
            }
        }

        /////////////////////////////////////////////////////////
        // Grab the second stone

        if(useDistanceSensorsALot){
            drive.newMoveToDistance(drive.frontLeftDistance, 5,1500,0 , true,4000);
            drive.newAccelerateInchesFB(2200,7,0,5000);
            liftSystem.grabAndStowNoWait(4500);
            teamUtil.sleep(750);
            drive.newAccelerateInchesFB(-2200,(RED ? 5 : 5/*TODO*/),0,5000);

        } else {
            drive.newAccelerateInchesFB(2200,11,0,5000);
            liftSystem.grabAndStowNoWait(4500);
            teamUtil.sleep(750);
            drive.newAccelerateInchesFB(-2200,(RED ? 7 : 7/*TODO*/),0,5000);
        }

        //strafe a tad to avoid collision with the wall when rotating towards building site
        if(path == 2){
            if(RED){
                drive.moveInchesRight(0.5, 4, 2000);
            } else {
                drive.moveInchesLeft(0.5, 4, 2000);
            }
        }
        drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_BUILDING);

        /////////////////////////////////////////////////////////
        // Deliver the second stone
        switch (path) { // TODO: OR, we could go back to finding the tape line as we cross it and moving a set distance from there...
            case 3 : distance = (RED ? 66.5  :66.5/*TODO*/); break;
            case 2 : distance = (RED ? 70.5  :70.5/*TODO*/); break;// TODO RED + 8?
            case 1 : distance = (RED ? 60.5  :60.5/*TODO*/); break;// TODO RED + 8?
        }
        drive.newAccelerateInchesFB(2200,distance, RED ? 268: 92,5000);

        liftSystem.lift.slightlyMoveLiftBaseUp(1, 2000);
        liftSystem.grabber.slightlyOpenGrabber();
        teamUtil.sleep(750);
        liftSystem.lift.moveLiftBaseDownNoWait(0.5, 3000);
        teamUtil.sleep(500);
        while (!drive.bottomColor.isOnTape()) {
            drive.driveBackward(0.75);
        }
        drive.stopMotors();
        /////////////////////////////////////////////////////////
        // Park












/*        drive.accelerateToSpeedRight(0, 0.75);
        while (!drive.bottomColor.isOnTape()) {
            drive.driveRight(0.75);
        }
        drive.decelerateInchesRight(0.75, 6);
        liftSystem.grabber.slightlyOpenGrabber();
        drive.rotateToZero();
        teamUtil.sleep(750);
        drive.accelerateToSpeedLeft(0.35, 1);
        while (!drive.bottomColor.isOnTape()) {
            drive.driveLeft(1);
        }

        if (path == 3) {
            drive.decelerateInchesLeft(1, 44);
        } else if (path == 2) {
            drive.decelerateInchesLeft(1, 50);
        } else if (path == 1) {
            drive.stopMotors();
            teamUtil.log("path 1, stopping motors");
            return;
        }


        liftSystem.prepareToGrabNoWait(4000);
        drive.rotateToZero();
        drive.moveToDistance(drive.frontLeftDistance, 9, 0.45, 3000);
        while(!liftSystem.preparedToGrab){
            teamUtil.sleep(100);
        }
        drive.moveToDistance(drive.frontLeftDistance, 6, 0.3, 3000);
        drive.accelerateInchesForward(0.65, 7, 2000);
        liftSystem.grabAndStowNoWait(4500);
        teamUtil.sleep(750);

        if(path == 3){
            drive.moveInchesBackward(0.35, 9, 3000); //TODO: this didn't work on path 3...? --> fixed it, awaiting next download 1/10/19 4:12 PM
        } else if(path == 2){
            drive.moveInchesBackward(0.35, 10, 3000);
        }
        drive.rotateToZero();
        drive.accelerateToSpeedRight(0, 1);
        while (!drive.bottomColor.isOnTape()) {
            drive.driveRight(1);
        }
        drive.decelerateInchesRight(1, 6);
        liftSystem.grabber.slightlyOpenGrabber();
        drive.rotateToZero();
        while (!drive.bottomColor.isOnTape()) {
            drive.driveLeft(0.75);
        }
        drive.stopMotors();
*/





    }


}


