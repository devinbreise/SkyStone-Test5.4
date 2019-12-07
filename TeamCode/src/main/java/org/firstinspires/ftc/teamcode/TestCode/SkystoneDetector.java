package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

import java.util.List;

public class SkystoneDetector {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private static final String VUFORIA_KEY = "AQfNUG7/////AAABmUE+GcnGE0LEkw6V6sBYnPdv0drO1QVLisryY2Kp9RhXImHEPLJJuIQaWyj3TKOYDB9P82rUavLg/jTofMcts0xLv8L5R4YfYDSZA4eJJMyEDPZxz6MSUXIpxhs7pof23wYX49SR5f/mvVq/qNOYb2DkpNSjTrMLTmyj0quYsA2LKS6C4zqbTr9XMQLGgmI9dYHV6Nk7HMcltcyB2ETUXPMew+bsp+UugBpt0VPjc9kW09Vy9ZGo9UncX7B/Gw73Kua6lUqqHvtfXpi3Sn2xJMcqWLHn5bxzr1xOwk9Co2kr8A3rU2gxpVzWMAnWHiWGWw9MY6GcIz6rJk+mu/e5jQeTTF08EK6ZXnzITpZQElx0";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;


    public SkystoneDetector(Telemetry theTelemetry, HardwareMap theHardwareMap) {
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
    }

    public void startTracking() {
        teamUtil.log("Detector -- start tracking");
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

            if (tfod != null) {
                teamUtil.log("Detector -- calling activate on tfod");

                tfod.activate();
            }
            /** Wait for the game to begin */
            telemetry.addData(">", "Press Play to start op mode");
            telemetry.update();

    }

    public void stopTracking() {
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

    public boolean middleIsSkystone(Recognition object) {
        return object.getLabel() == "Skystone" && getCenter(object) > 130;
    }

    public int detect() {

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                    Recognition firstObject = null;
                    Recognition secondObject = null;
                    for (Recognition recognition : updatedRecognitions) {
                        if (firstObject == null) {
                            firstObject = recognition;
                        } else if (secondObject == null) {
                            secondObject = recognition;
                        } else {
                            teamUtil.log("detect -- couldn't find objects");
                            return (1);

                        }
                    }

                    if (firstObject == null || secondObject == null) {
                        return 0;
                    }


                    if (rightMostIsSkystone(firstObject) && !middleIsSkystone(secondObject)) {
                        teamUtil.log("PATH 1");
                        return 1;
                    } else if (!rightMostIsSkystone(firstObject) && middleIsSkystone(secondObject)) {
                        teamUtil.log("PATH 2");
                        return 2;

                    } else if (!rightMostIsSkystone(firstObject) && !middleIsSkystone(secondObject)){
                        teamUtil.log("PATH 3");
                        return 3;
                    }
            }//else {teamUtil.log("detect -- no updated recognitions");}
        } else {
            teamUtil.log("detect -- tfod inactivated");
        }

        teamUtil.log("wtf am I doing");
        return -1;
    }

    public void reportPath() {
        telemetry.addData("path: ", detect());
    }

    public void reportStoneInformation() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                for (Recognition recognition : updatedRecognitions) {
//                    telemetry.addData("rightIsSky?: ", rightMostIsSkystone(recognition));
//                    telemetry.addData("middleIsSky?: ", middleIsSkystone(recognition));

                    telemetry.addData("label:", recognition.getLabel());

                    telemetry.addData("center: ", getCenter(recognition));

                }
            } //else {teamUtil.log("reportStoneInformation -- no updated recognitions");}
        } else {teamUtil.log("reportStoneInformation -- tfod inactivated");}
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




