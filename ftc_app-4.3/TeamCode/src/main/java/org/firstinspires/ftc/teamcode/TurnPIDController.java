package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

public class TurnPIDController extends PIDController {
    /**
     * Instantiate a new PIDController.
     *
     * @param kP
     * @param kI
     * @param kD
     * @param tolerance
     * @param opmode
     */
    public TurnPIDController(double kP, double kI, double kD, double tolerance, OpMode opmode) {
        super(kP, kI, kD, tolerance, opmode);
    }

    @Override
    public double getOutput(double currentPosition) {
        currentError = setpoint - currentPosition;
        while (currentError > 180 && ((LinearOpMode)opMode).opModeIsActive()) currentError -= 360;
        while (currentError <= -179 && ((LinearOpMode)opMode).opModeIsActive()) currentError += 360;
        sumError += currentError;

        double currentTime = opMode.getRuntime();
        double rateOfChange = (currentError - prevError) / (currentTime - prevTime);

        prevError = currentError;
        prevTime = currentTime;
        return Range.clip(kP * currentError + kI * sumError + kD * rateOfChange, -1.0, 1.0);
    }

    @Override
    public void reset(double setpoint, double startingPos) {
        this.setpoint = setpoint;
        running = true;

        sumError = 0;
        currentError = setpoint - startingPos;
        while (currentError > 180 && ((LinearOpMode)opMode).opModeIsActive()) currentError -= 360;
        while (currentError <= -179 && ((LinearOpMode)opMode).opModeIsActive()) currentError += 360;
        prevError = currentError;
        prevTime = opMode.getRuntime();
    }
}
