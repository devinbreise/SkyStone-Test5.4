package org.firstinspires.ftc.teamcode.basicLibs;

import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;


/**
 * {@link com.qualcomm.hardware.stmicroelectronics.VL53L0X} implements support for the STMicroelectronics VL53L0x time-of-flight distance sensor.
 *
 * @see <a href="http://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html">STMicroelectronics VL53L0X Sensor</a>
 *
 */
@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
@I2cDeviceType
@DeviceProperties(name = "@string/team_laser_sensor_name", description = "@string/team_laser_sensor_name", xmlTag = "TEAM_RANGE_SENSOR", compatibleControlSystems = ControlSystem.REV_HUB, builtIn = true)
public class teamDistanceSensor extends teamVL53L0X {
    public teamDistanceSensor(I2cDeviceSynch deviceClient) {
        super(deviceClient);
    }

    @Override
    public String getDeviceName() {
        return "TEAM 2M ToF Distance Sensor";
    }
}
