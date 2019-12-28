package org.firstinspires.ftc.teamcode.basicLibs;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class SkystoneDetector {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";
    private final int SKYSTONE_BOUNDARY_1 = 250;
    private final int SKYSTONE_BOUNDARY_2 = 475;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private static final String VUFORIA_KEY = "AQfNUG7/////AAABmUE+GcnGE0LEkw6V6sBYnPdv0drO1QVLisryY2Kp9RhXImHEPLJJuIQaWyj3TKOYDB9P82rUavLg/jTofMcts0xLv8L5R4YfYDSZA4eJJMyEDPZxz6MSUXIpxhs7pof23wYX49SR5f/mvVq/qNOYb2DkpNSjTrMLTmyj0quYsA2LKS6C4zqbTr9XMQLGgmI9dYHV6Nk7HMcltcyB2ETUXPMew+bsp+UugBpt0VPjc9kW09Vy9ZGo9UncX7B/Gw73Kua6lUqqHvtfXpi3Sn2xJMcqWLHn5bxzr1xOwk9Co2kr8A3rU2gxpVzWMAnWHiWGWw9MY6GcIz6rJk+mu/e5jQeTTF08EK6ZXnzITpZQElx0";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;


    public SkystoneDetector(Telemetry theTelemetry, HardwareMap theHardwareMap) {
        teamUtil.log("Constructing Detector");
        this.telemetry = theTelemetry;
        this.hardwareMap = theHardwareMap;

    }

    public void initDetector() {
        teamUtil.log("Initializing Detector");
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        teamUtil.log("Initializing Detector - Finished");
    }

    public void activateDetector() {
        teamUtil.log("Detector -- Activating");
            if (tfod != null) {
                teamUtil.log("Detector -- calling activate on tfod");
                tfod.activate();
            }
    }

    public void shutdownDector() {
        if (tfod != null) {
            teamUtil.log("Detector -- calling shutdown on tfod");
            tfod.shutdown();
        }

    }

    public double getCenter(Recognition object) {
        return (object.getRight() - object.getLeft()) / 2 + object.getLeft();
    }

    public boolean rightMostIsSkystone(Recognition object) {
        return object.getLabel() == "Skystone" && getCenter(object) > 350;
    }

    public boolean leftMostIsSkystone(Recognition object) {
        return object.getLabel() == "Skystone" && getCenter(object) > 130;
    }

    public int detectRed() {
        teamUtil.log("Detect");

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            String logString = "Detected:";
            if (updatedRecognitions != null) {
                    logString = logString+updatedRecognitions.size()+":";
                    //Recognition firstObject = null;
                    //Recognition secondObject = null;
                    for (Recognition recognition : updatedRecognitions) {
                         logString = logString+ recognition.getLabel() + " C:"+ getCenter(recognition) + " / ";

                        if(recognition.getLabel()== LABEL_SKYSTONE){
                            teamUtil.log("Skystone: " + recognition.getLeft() + ":" + recognition.getRight() + ":" + getCenter(recognition));
                            if(getCenter(recognition)< 150){

                                return 2;
                            }else {
                                return 3;
                            }

                        }

                    }

                return 1;
            } else {
                teamUtil.log("detectRed -- no updated recognitions");
                return -1;
            }
        } else {
            teamUtil.log("detectRed -- tfod inactivated");
            return -1;
        }

    }

    public int detectBlue() {
        teamUtil.log("Detect");

        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            String logString = "Detected:";
            if (updatedRecognitions != null) {
                logString = logString+updatedRecognitions.size()+":";
                //Recognition firstObject = null;
                //Recognition secondObject = null;
                for (Recognition recognition : updatedRecognitions) {
                    logString = logString+ recognition.getLabel() + " C:"+ getCenter(recognition) + " / ";

                    // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.

                    if(recognition.getLabel()== LABEL_SKYSTONE){
                        if(getCenter(recognition)> 350){
                            teamUtil.log("Detect 1 - Finished");
                            return 1;
                        }else if(getCenter(recognition) > 125){
                            if(recognition.getWidth() < 300) {
                                teamUtil.log("Detect 2 - Finished");
                                return 2;
                            }else {
                                return 3;
                            }
                        }

                    }
                }
                teamUtil.log(logString);

 /*                   if (rightMostIsSkystone(firstObject)) {
                        teamUtil.log("PATH 1");
                        return 1;
                    } else if (leftMostIsSkystone(secondObject)) {
                        teamUtil.log("PATH 2");
                        return 2;

                    } else if (!rightMostIsSkystone(firstObject) && !leftMostIsSkystone(secondObject)){
                        teamUtil.log("PATH 3");
                        return 3;
                    }
*/
            } else {
                teamUtil.log("detectBlue -- no updated recognitions");
                return -1;
            }
        } else {
            teamUtil.log("detectBlue -- tfod inactivated");
            return -1;
        }

        teamUtil.log("Detect - Finished");
        return -1;
    }


    public void reportPath() {
        telemetry.addData("path: ", detectRed());
    }

    public boolean reportStoneInformation() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                for (Recognition recognition : updatedRecognitions) {
//                    telemetry.addData("rightIsSky?: ", rightMostIsSkystone(recognition));
//                    telemetry.addData("middleIsSky?: ", leftMostIsSkystone(recognition));

                    teamUtil.log("label:" + recognition.getLabel());

                    teamUtil.log("center: " + getCenter(recognition));
                    if(recognition.getLabel()==LABEL_SKYSTONE){
                        return true;
                    }

                }
            } //else {teamUtil.log("reportStoneInformation -- no updated recognitions");}
        } else {teamUtil.log("reportStoneInformation -- tfod inactivated");}
        return false;
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
    }
}




