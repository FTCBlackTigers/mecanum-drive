package org.firstinspires.ftc.teamcode.controller;

public class Button {
    private boolean state;
    private boolean prevState;

    public Button() {
        state = false;
        prevState = false;
    }

    void setState(boolean state){
        this.state = state;
    }

    void setPrevState(){
        this.prevState = this.state;
    }

    public boolean isPressed(){
        return state;
    }

    public boolean onClick(){
        return this.state && !this.prevState;
    }

    public boolean onRealese(){
        return !this.state && this.prevState;
    }
}
