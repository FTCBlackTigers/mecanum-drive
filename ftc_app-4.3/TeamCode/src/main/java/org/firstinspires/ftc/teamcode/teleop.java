package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "mecanum drive", group = "")
public class teleop extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void init() {
        mecanumDrive.init(hardwareMap, this);
    }

    @Override
    public void loop() {
        mecanumDrive.teleopMotion(gamepad1);
    }
}
