package org.firstinspires.ftc.teamcode.basicLibs;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TeamGamepad {
    private OpMode theOpMode;
    private boolean isPressed = false;
    private boolean wasBounced = false;

    public TeamGamepad(OpMode opmode){
        theOpMode = opmode;
    }

    public void gamepadLoop() {
        if (isPressed && !theOpMode.gamepad2.right_bumper){
            wasBounced = true;
            isPressed = false;
        }else if(theOpMode.gamepad2.right_bumper){
            isPressed = true;
        }
        if (isPressed && !theOpMode.gamepad2.b){
            wasBounced = true;
            isPressed = false;
        }else if(theOpMode.gamepad2.b){
            isPressed = true;
        }


    }

    public boolean gamepad2RightBumperBounced(){
        if(wasBounced){
            wasBounced = false;
            return true;
        } else {
            return false;
        }

    }

    public boolean gamepad2bBounced(){
        if(wasBounced){
            wasBounced = false;
            return true;
        } else {
            return false;
        }

    }

}
