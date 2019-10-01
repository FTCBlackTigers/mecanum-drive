package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

class MecanumDrive {

    public MecanumDrive(){}

    public enum DriveDirection {
        FORWARD,BACKWARD,RIGHT,LEFT
    }

    public class Motion{
        // Robot speed [-1, 1].
        public final double vD;
        // Robot angle while moving [0, 2pi].
        public final double thetaD;
        // Speed for changing direction [-1, 1].
        public final double vTheta;

        /**
         * Sets the motion to the given values.
         */
        public Motion(double vD, double thetaD, double vTheta) {
            this.vD = vD;
            this.thetaD = thetaD;
            this.vTheta = vTheta;
        }
    }

    public class Wheels{
        private double frontLeft;
        private double frontRight;
        private double backLeft;
        private double backRight;

        public double getFrontLeft() {
            return frontLeft;
        }

        public double getFrontRight() {
            return frontRight;
        }

        public double getBackLeft() {
            return backLeft;
        }

        public double getBackRight() {
            return backRight;
        }

        /**
         * Sets the wheels to the given values.
         */
        public Wheels(double frontLeft, double frontRight,
                      double backLeft, double backRight) {
            List<Double> powers = Arrays.asList(frontLeft, frontRight, backLeft, backRight);
            clampPowers(powers);

            this.frontLeft = powers.get(0);
            this.frontRight = powers.get(1);
            this.backLeft = powers.get(2);
            this.backRight = powers.get(3);
        }

        private void clampPowers(List<Double> powers) {
            double minPower = Collections.min(powers);
            double maxPower = Collections.max(powers);
            double maxMag = Math.max(Math.abs(minPower), Math.abs(maxPower));

            for (int i = 0; i < powers.size(); i++) {
                powers.set(i, powers.get(i) / maxMag);
            }
        }
        /**
         * Scales the wheel powers by the given factor.
         * @param scalar The wheel power scaling factor.
         */
        public void scaleWheelPower(double scalar) {
            frontLeft*=scalar;
            frontRight*=scalar;
            backLeft*=scalar;
            backRight*=scalar;
        }
    }

    public Motion joystickToMotion(Gamepad gamePad, double currentAngle){
        final double JOYSTICK_THRESHOLD = 0.2;
        double leftX = gamePad.left_stick_x;
        double leftY = -gamePad.left_stick_y;
        double rightX = gamePad.right_stick_x;

        leftX = Math.abs(leftX) < JOYSTICK_THRESHOLD ? 0 : leftX;
        leftY = Math.abs(leftY) < JOYSTICK_THRESHOLD ? 0 : leftY;
        rightX = Math.abs(rightX) < JOYSTICK_THRESHOLD ? 0 : rightX;

        double vD = Math.hypot(leftX, leftY);

        double thetaD = Math.atan2(leftX, leftY);
        double radCurrentAngle = Math.toRadians(currentAngle);
        //driving by driver's view
        thetaD += gamePad.right_trigger>JOYSTICK_THRESHOLD ? radCurrentAngle : 0;
        while (thetaD > Math.PI)  thetaD -=  Math.PI * 2;
        while (thetaD <= -Math.PI) thetaD +=  Math.PI * 2;

        double vTheta = rightX;

        return new Motion(vD, thetaD, vTheta);
    }


    private Wheels motionToWheels(Motion motion) {
        double vD = motion.vD;
        double thetaD = motion.thetaD;
        double vTheta = motion.vTheta;

        double frontLeft = vD * Math.sin(thetaD + Math.PI / 4) + vTheta;
        double frontRight = vD * Math.cos(thetaD + Math.PI / 4) - vTheta;
        double backLeft = vD * Math.cos(thetaD + Math.PI / 4) + vTheta;
        double backRight = vD * Math.sin(thetaD + Math.PI / 4) - vTheta;
        Wheels wheels = new Wheels(frontLeft, frontRight, backLeft, backRight);
        wheels.scaleWheelPower(vD > 0 ? Math.abs(vD) : Math.abs(vTheta));
        return wheels;
    }

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    private BT_Gyro gyro = new BT_Gyro();
    private TurnPIDController turnPID = null;


    private OpMode opMode;

    //TODO: check the numbers
    private static final double COUNTS_PER_MOTOR_REV = 28 ;
    private static final double DRIVE_GEAR_REDUCTION = 19.2 ;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_CM = 10.16 ;     // For figuring circumference
    private static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * Math.PI);

    public void init(HardwareMap hardwareMap, OpMode opMode){
        this.opMode = opMode;

        turnPID = new TurnPIDController(1, 0, 0, 2, this.opMode);

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        //TODO: check the dirs
        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gyro.init(hardwareMap);
    }

    public void teleopMotion(Gamepad driver){
        Motion motion = joystickToMotion(driver, gyro.getAngle());
        Wheels wheels = motionToWheels(motion);

        this.frontLeftDrive.setPower(wheels.getFrontLeft());
        this.frontRightDrive.setPower(wheels.getFrontRight());
        this.backLeftDrive.setPower(wheels.getBackLeft());
        this.backRightDrive.setPower(wheels.getBackRight());
    }

    public void driveByEncoder(double speed, double distCm, DriveDirection direction, double timeoutS) {
        int newFrontLeftTarget = frontLeftDrive.getCurrentPosition();
        int newFrontRightTarget = frontRightDrive.getCurrentPosition();
        int newBackLeftTarget = backLeftDrive.getCurrentPosition();
        int newBackRightTarget = backRightDrive.getCurrentPosition();

        // Determine new target position, and pass to motor controller
        int ticks = (int)(distCm / Math.cos(Math.PI/4) * COUNTS_PER_CM);
        switch (direction){
            case FORWARD:
                newFrontLeftTarget += ticks ;
                newFrontRightTarget += ticks;
                newBackLeftTarget += ticks;
                newBackRightTarget += ticks;
                break;
            case BACKWARD:
                newFrontLeftTarget -= ticks ;
                newFrontRightTarget -= ticks;
                newBackLeftTarget -= ticks;
                newBackRightTarget -= ticks;
                break;
            case RIGHT:
                newFrontLeftTarget -= ticks ;
                newFrontRightTarget += ticks;
                newBackLeftTarget += ticks;
                newBackRightTarget -= ticks;
                break;
            case LEFT:
                newFrontLeftTarget += ticks ;
                newFrontRightTarget -= ticks;
                newBackLeftTarget -= ticks;
                newBackRightTarget += ticks;
                break;
        }
        opMode.telemetry.addData("front left target: " , ticks + ", " + newFrontLeftTarget);
        opMode.telemetry.addData("front right target: " , ticks + ", " + newFrontRightTarget);
        opMode.telemetry.addData("rear left target: " , ticks + ", " + newBackLeftTarget);
        opMode.telemetry.addData("rear right target: " , ticks + ", " + newBackRightTarget);
        opMode.telemetry.update();

        frontLeftDrive.setTargetPosition(newFrontLeftTarget);
        frontRightDrive.setTargetPosition(newFrontRightTarget);
        backLeftDrive.setTargetPosition(newBackLeftTarget);
        backRightDrive.setTargetPosition(newBackRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // calculate the stop time and start motion.
        double timeToStop = this.opMode.getRuntime() + timeoutS;
        frontLeftDrive.setPower(Math.abs(speed));
        frontRightDrive.setPower(Math.abs(speed));
        backLeftDrive.setPower(Math.abs(speed));
        backRightDrive.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((opMode.getRuntime() < timeToStop) && (((LinearOpMode)opMode).opModeIsActive()) &&
                (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && backLeftDrive.isBusy() && backRightDrive.isBusy())) {
            opMode.telemetry.addData("front left: " ,newFrontLeftTarget + " , " + frontLeftDrive.getCurrentPosition());
            opMode.telemetry.addData("front right: " ,newFrontRightTarget + " , " + frontRightDrive.getCurrentPosition());
            opMode.telemetry.addData("rear left: " ,newBackLeftTarget + " , " + backLeftDrive.getCurrentPosition());
            opMode.telemetry.addData("rear right: " ,newBackRightTarget + " , " + backRightDrive.getCurrentPosition());
            opMode.telemetry.update();
        }

        // Stop all motion;
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveByEncoder(double distant, double angle, double speed){
        Motion motion = new Motion(Math.abs(speed), Math.toRadians(angle), 0);

        Wheels wheelsPowers = motionToWheels(motion);
        int ticks = (int)(distant / Math.cos(Math.PI/4) * COUNTS_PER_CM);

        int newFrontLeftTarget =  frontLeftDrive.getCurrentPosition() + (int)(ticks * wheelsPowers.getFrontLeft());
        int newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(ticks * wheelsPowers.getFrontRight());
        int newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int)(ticks * wheelsPowers.getBackLeft());
        int newBackRightTarget = backRightDrive.getCurrentPosition() + (int)(ticks * wheelsPowers.getBackRight());

        opMode.telemetry.addData("front left target: " , ticks + ", " + newFrontLeftTarget);
        opMode.telemetry.addData("front right target: " , ticks + ", " + newFrontRightTarget);
        opMode.telemetry.addData("rear left target: " , ticks + ", " + newBackLeftTarget);
        opMode.telemetry.addData("rear right target: " , ticks + ", " + newBackRightTarget);
        opMode.telemetry.update();

        frontLeftDrive.setTargetPosition(newFrontLeftTarget);
        frontRightDrive.setTargetPosition(newFrontRightTarget);
        backLeftDrive.setTargetPosition(newBackLeftTarget);
        backRightDrive.setTargetPosition(newBackRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        frontLeftDrive.setPower(-Math.abs(wheelsPowers.getFrontLeft()));
        frontRightDrive.setPower(-Math.abs(wheelsPowers.getFrontRight()));
        backLeftDrive.setPower(-Math.abs(wheelsPowers.getBackLeft()));
        backRightDrive.setPower(-Math.abs(wheelsPowers.getBackRight()));

        while (((LinearOpMode)opMode).opModeIsActive() &&
                (frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy()) ){
            opMode.telemetry.addData("front left: " ,newFrontLeftTarget + " , " + frontLeftDrive.getCurrentPosition());
            opMode.telemetry.addData("front right: " ,newFrontRightTarget + " , " + frontRightDrive.getCurrentPosition());
            opMode.telemetry.addData("rear left: " ,newBackLeftTarget + " , " + backLeftDrive.getCurrentPosition());
            opMode.telemetry.addData("rear right: " ,newBackRightTarget + " , " + backRightDrive.getCurrentPosition());
            opMode.telemetry.update();
        }


        // Stop all motion;
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void turnByGyroAbsolut(double targetDegree, double timeS) {
        turnPID.reset(targetDegree, gyro.getAngle());
        timeS += opMode.getRuntime();

        while (!turnPID.onTarget() && ((LinearOpMode)opMode).opModeIsActive() && opMode.getRuntime() <= timeS) {
            while (!turnPID.onTarget() && ((LinearOpMode)opMode).opModeIsActive()&& opMode.getRuntime() <= timeS) {
                double output = turnPID.getOutput(gyro.getAngle());
                frontLeftDrive.setPower(-output);
                backLeftDrive.setPower(-output);
                frontRightDrive.setPower(output);
                backRightDrive.setPower(output);
                opMode.telemetry.addData("error: ", turnPID.getCurrentError())
                        .addData("output: ", output)
                        .addData("robot angle: ", gyro.getAngle());
                opMode.telemetry.update();
            }
            double time = opMode.getRuntime() + 0.3;
            while ((opMode.getRuntime() < time) && ((LinearOpMode)opMode).opModeIsActive()) {
                turnPID.updateError(gyro.getAngle());
            }
        }

        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void turnByGyroRelative(double degrees, double timeS) {
        turnByGyroAbsolut(this.gyro.getAngle() + degrees, timeS);
    }

    /*public void autonumousDrive(double distane, double driveAngle, double endAngle){
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.drivePID.reset(distane,0);
        this.turnPID.reset(endAngle, gyro.getAngle());

        while(((LinearOpMode)this.opMode).opModeIsActive() &&
                (!drivePID.onTarget() || !turnPID.onTarget())){
            //Motion motion = new Motion()
            //Wheels wheels = motionToWheels()
        }
    }*/
}
