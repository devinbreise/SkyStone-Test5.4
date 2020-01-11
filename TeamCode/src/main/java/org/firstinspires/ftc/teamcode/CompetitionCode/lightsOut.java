package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Lights Off")
@Disabled
public class lightsOut extends LinearOpMode {

    public void initialize() {
        teamUtil.init(this);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
    }

    @Override
    public void runOpMode() throws InterruptedException {


        teamUtil.telemetry.addLine("Initializing Op Mode...please wait");
        teamUtil.telemetry.update();
        initialize();
        teamUtil.telemetry.addLine("Lights Off");
        teamUtil.telemetry.update();
        waitForStart();
    }
}
