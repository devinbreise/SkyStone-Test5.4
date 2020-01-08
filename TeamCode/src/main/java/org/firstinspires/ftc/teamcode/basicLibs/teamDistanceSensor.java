package org.firstinspires.ftc.teamcode.basicLibs;

import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;


@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
@I2cDeviceType
@DeviceProperties(name = "Team Distance Sensor", description = "Team Distance Sensor", xmlTag = "TEAM_RANGE_SENSOR", compatibleControlSystems = ControlSystem.REV_HUB, builtIn = true)
public class teamDistanceSensor extends teamVL53L0X {
    public teamDistanceSensor(I2cDeviceSynch deviceClient) {
        super(deviceClient);
    }

    @Override
    public String getDeviceName() {
        return "TEAM 2M ToF Distance Sensor";
    }
}
