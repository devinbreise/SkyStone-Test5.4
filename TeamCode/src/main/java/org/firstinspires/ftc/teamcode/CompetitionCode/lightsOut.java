package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name="Lights Off")
public class lightsOut extends LinearOpMode {

    public void initialize() {
        teamUtil.init(this);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
    }

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addLine("Initializing Op Mode...please wait");
        telemetry.update();
        initialize();
        telemetry.addLine("Lights Off");
        telemetry.update();
        waitForStart();
    }
}
