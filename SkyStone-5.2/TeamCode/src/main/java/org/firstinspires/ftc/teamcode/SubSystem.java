package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class SubSystem {
    protected OpMode opMode;

    public abstract void init(HardwareMap hardwareMap, OpMode opMode);
    public abstract void teleopMotion(Gamepad driver, Gamepad operator);
}
