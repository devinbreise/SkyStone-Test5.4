package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class Lift {

    private double liftBasePower = 1;
    private DcMotor liftBase;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Lift(HardwareMap theHardwareMap, Telemetry theTelmetry){
        hardwareMap = theHardwareMap;
        telemetry = theTelmetry;

    }

    public void initLift(){
        liftBase = hardwareMap.dcMotor.get("liftBase");
    }

    public void liftUp(){
        liftBase.setPower(liftBasePower);


    }

    public void liftDown(){
        liftBase.setPower(-liftBasePower);


    }

    public void shutDownLift(){
        liftBase.setPower(0);
    }

    public void increaseLiftPower(){
        liftBasePower = liftBasePower + 0.1;
        teamUtil.sleep(500);


    }
    public void decreaseLiftPower(){

        liftBasePower = liftBasePower - 0.1;
        teamUtil.sleep(200);

    }

    public void liftTelemetry(){
        telemetry.addData("Motor Power:", liftBasePower);
    }





}
