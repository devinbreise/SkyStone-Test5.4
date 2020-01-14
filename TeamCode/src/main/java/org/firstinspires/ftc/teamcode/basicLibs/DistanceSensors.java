package org.firstinspires.ftc.teamcode.basicLibs;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//This class is used for to control the distance sensors by using the reading's of the sensors and checking if the readings are valid.
//This code can be used for using the distance sensor to go to a certain distance or square up on the wall or just to get a valid reading of the distance sensors.
//

public class DistanceSensors {
    HardwareMap hardwareMap;
    private Rev2mDistanceSensor rev2mDistanceSensor;//This is the distance sensor variable
    private float offset;


    public DistanceSensors(Rev2mDistanceSensor theDistanceSensor) {
            rev2mDistanceSensor = theDistanceSensor;
    }

    public void setOffset(float theOffset){
        offset = theOffset;
    }

    //This method return's the current reading of the distance sensors and returns the value.
    public double getDistance() {//find the value that will be mostly used in the autonomous
        double distance = rev2mDistanceSensor.getDistance(DistanceUnit.INCH);//This get the current reading of the distance sensor
        return distance + offset;//This return the value of the current distance sensor minus whatever offset has been set for this sensor


    }//This method check's if the reading is valid and the distance sensor's are detecting a vaild reading under the point where it does not read the correct distance.
    public boolean validReading(){ //find the value that will be mostly used in the autonomous
        //This see's if the current distance is less than 20
        //This returns the false value if the number uis greater than 20
        return getDistance() < 20;//If the current reating is less thean 20 it will return the value as a valid reading

    }

}
















