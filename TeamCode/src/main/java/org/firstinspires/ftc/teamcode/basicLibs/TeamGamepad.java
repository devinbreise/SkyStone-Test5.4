package org.firstinspires.ftc.teamcode.basicLibs;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TeamGamepad {
    private OpMode theOpMode;
    private boolean isPressedRB = false;
    private boolean wasBouncedRB = false;
    private boolean isPressedDPD = false;
    private boolean wasBouncedDPD = false;
    private boolean isPressedDPU = false;
    private boolean wasBouncedDPU = false;

    public TeamGamepad(OpMode opmode){
        theOpMode = opmode;
    }

    public void gamepadLoop() {
        if(theOpMode.gamepad2.right_bumper){
            isPressedRB = true;
        }else if (isPressedRB && !theOpMode.gamepad2.right_bumper){
            wasBouncedRB = true;
            isPressedRB = false;
        }

        if(theOpMode.gamepad2.dpad_up){
            isPressedDPD = true;
        }else if (isPressedDPD && !theOpMode.gamepad2.dpad_up){
            wasBouncedDPD = true;
            isPressedDPD = false;
        }

        if(theOpMode.gamepad2.dpad_down){
            isPressedDPU = true;
        }else if (isPressedDPU && !theOpMode.gamepad2.dpad_down){
            wasBouncedDPU = true;
            isPressedDPU = false;
        }


    }

    public boolean gamepad2RightBumperBounced(){
        if(wasBouncedRB){
            wasBouncedRB = false;
            return true;
        } else {
            return false;
        }

    }

    public boolean gamepad2dpad_upBounced(){
        if(wasBouncedDPD){
            wasBouncedDPD = false;
            return true;
        } else {
            return false;
        }

    }
    public boolean gamepad2dpad_downBounced(){
        if(wasBouncedDPU){
            wasBouncedDPU = false;
            return true;
        } else {
            return false;
        }

    }

}
