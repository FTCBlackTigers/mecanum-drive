package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.Controller;


@TeleOp(name = "mecanum drive", group = "")
public class MecanumTeleop extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Controller driver = new Controller();

    MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void init() {
        mecanumDrive.init(hardwareMap, this);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        mecanumDrive.teleopMotion(gamepad1, gamepad2);
    }
}
