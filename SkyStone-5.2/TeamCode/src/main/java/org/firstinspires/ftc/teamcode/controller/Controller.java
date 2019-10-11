package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    public Button a = new Button();
    public Button b = new Button();
    public Button x = new Button();
    public Button y = new Button();

    public Button rightBumber = new Button();
    public Button leftBumper = new Button();

    public Button dpadUp = new Button();
    public Button dpadDown = new Button();
    public Button dpadRight = new Button();
    public Button dpadLeft = new Button();

    public Button start = new Button();
    public Button back = new Button();
    public Button guide = new Button();

    public Button leftStickButton = new Button();
    public Button rightStickButton = new Button();

    public Axis rightTrigger = new Axis(false, 0.0);
    public Axis leftTrigger = new Axis(false, 0.0);

    public Axis leftStickX = new Axis(false, 0.0);
    public Axis leftStickY = new Axis(true, 0.0);
    public Axis rightStickX = new Axis(false, 0.0);
    public Axis rightStickY = new Axis(true, 0.0);

    public Controller() {
    }

    public void setValues(Gamepad gamepad){
        a.setState(gamepad.a);
        b.setState(gamepad.b);
        x.setState(gamepad.x);
        y.setState(gamepad.y);

        rightBumber.setState(gamepad.right_bumper);
        leftBumper.setState(gamepad.left_bumper);

        dpadUp.setState(gamepad.dpad_up);
        dpadDown.setState(gamepad.dpad_down);
        dpadLeft.setState(gamepad.dpad_left);
        dpadRight.setState(gamepad.dpad_right);

        start.setState(gamepad.start);
        back.setState(gamepad.back);
        guide.setState(gamepad.guide);

        leftStickButton.setState(gamepad.left_stick_button);
        rightStickButton.setState(gamepad.right_stick_button);

        rightTrigger.setValue(gamepad.right_trigger);
        leftTrigger.setValue(gamepad.left_trigger);

        leftStickX.setValue(gamepad.left_stick_x);
        leftStickY.setValue(gamepad.left_stick_y);

        rightStickX.setValue(gamepad.right_stick_x);
        rightStickY.setValue(gamepad.right_stick_y);
    }

    public void setPrevValues(){
        a.setPrevState();
        b.setPrevState();
        x.setPrevState();
        y.setPrevState();

        rightBumber.setPrevState();
        leftBumper.setPrevState();

        dpadUp.setPrevState();
        dpadDown.setPrevState();
        dpadLeft.setPrevState();
        dpadRight.setPrevState();

        start.setPrevState();
        back.setPrevState();
        guide.setPrevState();

        leftStickButton.setPrevState();
        rightStickButton.setPrevState();

        rightTrigger.setPrevValue();
        leftTrigger.setPrevValue();

        leftStickX.setPrevValue();
        leftStickY.setPrevValue();

        rightStickX.setPrevValue();
        rightStickY.setPrevValue();
    }
}
