package org.firstinspires.ftc.teamcode.Assemblies;

        import com.qualcomm.robotcore.hardware.HardwareMap;

        import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public LiftSystem liftSystem;
    public RobotDrive drive;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Robot(Telemetry theTelemetry, HardwareMap theHardwareMap){
        telemetry = theTelemetry;
        hardwareMap = theHardwareMap;

        liftSystem = new LiftSystem(hardwareMap, telemetry);
        drive = new RobotDrive(hardwareMap, telemetry);
    }

    public void init(){
        drive.initImu();
        drive.initDriveMotors();
        drive.initDistanceSensors();
        drive.resetHeading();
        liftSystem.initLiftSystem();
    }

    public void autoIntake(boolean isBlue){
        if(isBlue){
            liftSystem.prepareToGrabNoWait();
            //drive.imuRotateToAngle(180);
            drive.frontLeftCloseToDistance(3, 0.3);
            do{
                drive.driveLeft(0.3);
            }while(drive.frontLeftDistance.getDistance()<10);
            drive.moveInchesLeft(.3, 2);
            liftSystem.grabAndStow("wide");
        }else{

        }
    }
}
