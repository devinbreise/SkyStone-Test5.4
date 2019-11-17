package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class Lift {

    private double liftBasePower = .5;
    private DcMotor liftBase;
    private DcMotor rSpindle;
    private DcMotor lSpindle;
    //NEVEREST20_ENCODER_CLICKS = 537.6
    private RevTouchSensor liftDownLimit;

    // Keep track of when controls are updated to limit how fast values can change
    private long nextControlUpdate = System.currentTimeMillis();
    private final long CONTROL_INTERVAL = 500;

    // Constants for the spindles
    private final int LEVEL_0 = 340;
    private final int LEVEL_INCREMENT = 570;
    private final double TENSION_POWER = 0.05;


    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Lift(HardwareMap theHardwareMap, Telemetry theTelmetry){
        hardwareMap = theHardwareMap;
        telemetry = theTelmetry;

    }

    public void initLift(){
        liftBase = hardwareMap.dcMotor.get("liftBase");
        liftBase.setDirection(DcMotorSimple.Direction.REVERSE);

        rSpindle = hardwareMap.dcMotor.get("rSpindle");
        lSpindle = hardwareMap.dcMotor.get("lSpindle");
        lSpindle.setDirection(DcMotorSimple.Direction.REVERSE);
        rSpindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lSpindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftDownLimit = hardwareMap.get(RevTouchSensor.class, "liftDownLimitSwitch");
        //liftDownLimit.setMode(DigitalChannel.Mode.INPUT);
    }

    // These 5 methods are for testing...they are intended to give you button control
    // over the lift (e.g. while a button is held down, call liftUp, when the button is let go
    // call shutDownLift()
    public void shutDownLift(){
        liftBase.setPower(0);
    }
    public void liftUp(){
        liftBase.setPower(liftBasePower);
    }
    public void liftDown(){
        if(liftDownLimit.isPressed()){
            shutDownLift();
        } else {
            liftBase.setPower(-liftBasePower);
        }
    }
    public void increaseLiftPower(){
        // If enough time has passed since we last updated the power
        if (System.currentTimeMillis() > nextControlUpdate)
        {
            liftBasePower = liftBasePower + 0.1;
            nextControlUpdate = System.currentTimeMillis() + CONTROL_INTERVAL;
        }
    }
    public void decreaseLiftPower(){
        // If enough time has passed since we last updated the power
        if (System.currentTimeMillis() > nextControlUpdate)
        {
            liftBasePower = liftBasePower - 0.1;
            nextControlUpdate = System.currentTimeMillis() + CONTROL_INTERVAL;
        }
    }

    // we need to figure out where this sort of code should be run as the lift is
    // put through various operations
    public void tensionLiftString(){
        rSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSpindle.setPower(TENSION_POWER);
        lSpindle.setPower(TENSION_POWER);
        teamUtil.sleep(2000); // for testing, this is not OK in an op mode
        rSpindle.setPower(0);
        lSpindle.setPower(0);
        rSpindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Move the lift up to the specified level using FTC PID controller
    public void goToLevel(int level) {
        rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSpindle.setTargetPosition(LEVEL_0 + (LEVEL_INCREMENT * level));
        lSpindle.setTargetPosition(LEVEL_0 + (LEVEL_INCREMENT * level));
        rSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rSpindle.setPower(.99);
        lSpindle.setPower(.99);
    }

    // Drop the lift to its lowest point
    public void goToBottom() {
        rSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSpindle.setPower(0);
        lSpindle.setPower(0);
    }

    public void liftTelemetry(){
        telemetry.addData("Lift Motor Power:", liftBasePower);
        telemetry.addData("Base:", liftBase.getCurrentPosition());
        telemetry.addData("liftDownLimit:", liftDownLimit.isPressed());
        telemetry.addData("lSpindle:", lSpindle.getCurrentPosition());
        telemetry.addData("rSpindle:", rSpindle.getCurrentPosition());
    }
}
