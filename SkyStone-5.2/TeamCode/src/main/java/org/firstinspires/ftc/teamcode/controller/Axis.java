package org.firstinspires.ftc.teamcode.controller;

public class Axis {
    private double value;
    private double prevValue;
    private boolean inverted;
    private double threshold;

    public Axis(boolean inverted, double threshold) {
        this.inverted = inverted;
        this.threshold = Math.abs(threshold);
        this.value = 0.0;
        this.prevValue = 0.0;
    }

    void setValue(double value) {
        this.value = inverted? -1 : 1 * value;
    }

    void setPrevValue() {
        this.prevValue = this.value;
    }

    public boolean isPressed(){
        return Math.abs(this.value) > threshold;
    }

    public boolean onClick(){
        return Math.abs(this.value) > threshold && Math.abs(this.prevValue) <= threshold;
    }

    public boolean onRealese(){
        return Math.abs(this.value) <= threshold && Math.abs(this.prevValue) > threshold;
    }

    public double getValue() {
        return value;
    }
}
