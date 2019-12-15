package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "testPerformance")
@Disabled

public class TestPerformance extends OpMode {


    public void init() {
        teamUtil.initPerf();
    }

    public void loop() {
        teamUtil.trackPerf();
    }
}
