/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "mecanumDrive", group = "Concept")
@Disabled
public class ConceptNullOp extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();


  private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
  @Override
  public void init() {
    frontLeftDrive = hardworMap.get(DcMotor.class, "frontLeftDrive");
    frontRightDrive = hardworMap.get(DcMotor.class, "frontRightDrive");
    backLeftDrive = hardworMap.get(DcMotor.class, "backLeftDrive");
    backRightDrive = hardworMap.get(DcMotor.class, "backRightDrive");

    //TODO: check if right dirs
    frontLeftDrive.setDirction(DcMotor.Direction.FORWARD);
    frontRightDrive.setDirction(DcMotor.Direction.REVERSE);
    backLeftDrive.setDirction(DcMotor.Direction.FORWARD);
    backRightDrive.setDirction(DcMotor.Direction.REVERSE);

    frontLeftDrive.setPower(0);
    frontRightDrive.setPower(0);
    rearLeftDrive.setPower(0);
    rearRightDrive.setPower(0);

    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    telemetry.addData("Status", "Initialized");
  }

  @Override
  public void init_loop() {
  }


  @Override
  public void start() {
    runtime.reset();
  }


  @Override
  public void loop() {
    Motion motion = joystickToMotion(gamepad1, 0);
    Wheels wheels = motionToWheels(motion);
    frontLeftDrive.setPower(wheels.frontLeft);
    frontRightDrive.setPower(wheels.frontRight);
    backLeftDrive.setPower(wheels.backLeft);
    backRightDrive.setPower(wheels.backRight);
    telemetry.addData("Status", "Run Time: " + runtime.toString());
  }



  private class Motion{
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

  public  Motion joystickToMotion(GamePad gamePad, double currentAngle){
    final double JOYSTICK_THRESHOLD = 0.2;
    double leftX = gamePad.leftStick.x;
    double leftY = -gamePad.leftStick.y;
    double rightX = gamePad.rightStick.x;
    if (Math.abs(leftX) < JOYSTICK_THRESHOLD){
      leftX=0;
    }
    if (Math.abs(leftY) < JOYSTICK_THRESHOLD){
      leftY=0;
    }
    if (Math.abs(rightX) < JOYSTICK_THRESHOLD){
      rightX=0;
    }

    double vD = Math.min(Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), 1);//change to hypot method

    double thetaD = Math.atan2(leftX, leftY);
    double radAngle = curretAngle*Math.PI/180;
    //driving by driver's view
    //thetaD += radAngle;
    while (thetaD > Math.PI)  thetaD -=  Math.PI * 2;
    while (thetaD <= - Math.PI) thetaD +=  Math.PI * 2;

    double vTheta = rightX;

    return new Motion(vD, thetaD, vTheta);
  }


  public static class Wheels {
    // The mecanum wheels powers.
    public double frontLeft;
    public double frontRight;
    public double backLeft;
    public double backRight;
    /**
     * Sets the wheels to the given values.
     */
    public Wheels(double frontLeft, double frontRight,
                  double backLeft, double backRight) {
      List<Double> powers = Arrays.asList(frontLeft, frontRight,
              backLeft, backRight);
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

  public static Wheels motionToWheels(Motion motion) {
    double vD = motion.vD;
    double thetaD = motion.thetaD;
    double vTheta = motion.vTheta;

    double frontLeft = vD * Math.sin(thetaD + Math.PI / 4) + vTheta;
    double frontRight  = vD * Math.cos(thetaD + Math.PI / 4) - vTheta;
    double backLeft = vD * Math.cos(thetaD + Math.PI / 4) + vTheta;
    double backRight = vD * Math.sin(thetaD + Math.PI / 4) - vTheta;
    Wheels wheels = new Wheels(frontLeft, frontRight, backLeft, backRight);
    wheels.scaleWheelPower(vD>0?Math.abs(vD):Math.abs(vTheta));
    return wheels;
  }
}
