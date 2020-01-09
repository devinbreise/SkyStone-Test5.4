package org.firstinspires.ftc.teamcode.basicLibs;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TeamGamepad {
    private OpMode theOpMode;
    private boolean isPressedRB_2 = false;
    private boolean wasBouncedRB_2 = false;
    private boolean isPressedDPD_2 = false;
    private boolean wasBouncedDPD_2 = false;
    private boolean isPressedDPU_2 = false;
    private boolean wasBouncedDPU_2 = false;
    private boolean isPressedA_2 = false;
    private boolean wasBouncedA_2 = false;


    public TeamGamepad(OpMode opmode){
        theOpMode = opmode;
    }

    public void gamepadLoop() {
        if(theOpMode.gamepad2.right_bumper){
            isPressedRB_2 = true;
        }else if (isPressedRB_2 && !theOpMode.gamepad2.right_bumper){
            wasBouncedRB_2 = true;
            isPressedRB_2 = false;
        }

        if(theOpMode.gamepad2.dpad_up){
            isPressedDPD_2 = true;
        }else if (isPressedDPD_2 && !theOpMode.gamepad2.dpad_up){
            wasBouncedDPD_2 = true;
            isPressedDPD_2 = false;
        }

        if(theOpMode.gamepad2.dpad_down){
            isPressedDPU_2 = true;
        }else if (isPressedDPU_2 && !theOpMode.gamepad2.dpad_down){
            wasBouncedDPU_2 = true;
            isPressedDPU_2 = false;
        }

        if(theOpMode.gamepad2.a){
            isPressedA_2 = true;
        }else if (isPressedA_2 && !theOpMode.gamepad2.a){
            wasBouncedA_2 = true;
            isPressedA_2 = false;
        }


    }

    public boolean gamepad2RightBumperBounced(){
        if(wasBouncedRB_2){
            wasBouncedRB_2 = false;
            return true;
        } else {
            return false;
        }

    }

    public boolean gamepad2dpad_upBounced(){
        if(wasBouncedDPD_2){
            wasBouncedDPD_2 = false;
            return true;
        } else {
            return false;
        }

    }
    public boolean gamepad2dpad_downBounced(){
        if(wasBouncedDPU_2){
            wasBouncedDPU_2 = false;
            return true;
        } else {
            return false;
        }

    }
    public boolean gamepad2ABounced(){
        if(wasBouncedA_2){
            wasBouncedA_2 = false;
            return true;
        } else {
            return false;
        }

    }

}
