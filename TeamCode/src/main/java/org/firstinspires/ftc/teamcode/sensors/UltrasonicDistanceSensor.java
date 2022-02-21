package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.RobotConfig;
import org.firstinspires.ftc.teamcode.util.Subassembly;

public class UltrasonicDistanceSensor  implements Subassembly {
    private final OpMode opMode;
    private final HardwareMap hardwareMap;
    public final UltrasonicSensor usRangeSensor;

    public UltrasonicDistanceSensor(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;

        usRangeSensor = hardwareMap.get(UltrasonicSensor.class, RobotConfig.CURRENT.name("range_Rear"));
    }
}
