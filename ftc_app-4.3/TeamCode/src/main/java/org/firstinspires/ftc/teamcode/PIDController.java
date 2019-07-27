package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

public class PIDController {

    protected double setpoint, tolerance, currentError;
    protected double kP, kI, kD;
    protected OpMode opMode;

    protected double prevError, sumError, prevTime;
    protected boolean running = false;

    /**
     * Instantiate a new PIDController. 
     * @param kP
     * @param kI
     * @param kD
     * @param tolerance
     */
    public PIDController(double kP, double kI, double kD, double tolerance, OpMode opmode) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.tolerance = tolerance;
        this.opMode = opmode;
    }

    /**
     * Get the output from the PID controller.
     * @param currentPosition the current position of the mechanism
     * @return the output to be sent to the motor(s)
     */
    public double getOutput(double currentPosition) {
        currentError = setpoint - currentPosition;
        sumError += currentError;

        double currentTime = opMode.getRuntime();
        double rateOfChange = (currentError - prevError) / (currentTime - prevTime);
        
        prevError = currentError;
        prevTime = currentTime;
        return Range.clip(kP * currentError + kI * sumError + kD * rateOfChange, -1.0, 1.0);
    }

    /**
     * reset the PIDController
     * @param setpoint the target of the PIDController
     * @param startingPos the starting position of the mechanism
     */
    public void reset(double setpoint, double startingPos) {
        this.setpoint = setpoint;
        running = true;

        sumError = 0;
        currentError = setpoint - startingPos;
        prevError = currentError;
        prevTime = opMode.getRuntime();
    }

    public void stop() {
        running = false;
    }

    public boolean isRunning() {
        return running;
    }
    /**
     * 
     * @return true if the current position is within the given tolerance of the setpoint
     */
    public boolean onTarget() {
        return Math.abs(currentError) < tolerance;
    }

    /**
     * Set the P, I and D gains.
     * @param kP the kP to set
     * @param kI the kI to set
     * @param kD the kD to set
     */
    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * @return the kP
     */
    public double getkP() {
        return kP;
    }

    /**
     * @param kP the kP to set
     */
    public void setkP(double kP) {
        this.kP = kP;
    }

    /**
     * @return the kI
     */
    public double getkI() {
        return kI;
    }

    /**
     * @param kI the kI to set
     */
    public void setkI(double kI) {
        this.kI = kI;
    }

    /**
     * @return the kD
     */
    public double getkD() {
        return kD;
    }

    /**
     * @param kD the kD to set
     */
    public void setkD(double kD) {
        this.kD = kD;
    }

    /**
     * @param setpoint the setpoint to set
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getCurrentError() {
        return this.currentError;
    }

    public void updateError(double currentPos) {
        currentError = setpoint - currentPos;
    }
    public double getSetpoint() {
        return setpoint;
    }
}
