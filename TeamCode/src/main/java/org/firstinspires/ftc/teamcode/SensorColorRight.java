/* Copyright (c) 2017-2020 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Sensor: Color Right Corner", group = "Sensor")
public class SensorColorRight extends LinearOpMode {
  HardwarePushbotV3 robot   = new HardwarePushbotV3();
  private ElapsedTime     runtime = new ElapsedTime();

  static final double     LEG_TIME_1_start_1    = 1;
  static final double     LEG_TIME_1_start    = 1.65;
  // read color
  static final double     LEG_TIME_2_start    = 1;
  static final double     LEG_TIME_3_start    = 0.5;
  static final double     LEG_TIME_1_score    = 0;
  static final double     LEG_TIME_2_score    = 1.25;
  static final double     LEG_TIME_3_score    = 1.5;
  static final double     LEG_TIME_4_score    = 0.8;
  static final double     LEG_TIME_5_score    = 0.45;
  static final double     LEG_TIME_6_score    = 2;
  static final double     LEG_TIME_7_score    = 0.05;
  static final double     LEG_TIME_8_score    = 0.15;

  static final double     LEG_TIME_1_1    = 0.55;
  static final double     LEG_TIME_2_1   = 0.35;
  static final double     LEG_TIME_3_1   = 1;
  static final double     LEG_TIME_4_1  = 1;

  static final double     LEG_TIME_1_2    = 2.5;
  static final double     LEG_TIME_2_2   = 1;
  static final double     LEG_TIME_3_2   = 1;
  static final double     LEG_TIME_4_2  = 1;

  static final double     LEG_TIME_1_3    = 0.5;
  static final double     LEG_TIME_2_3   = 0.30;
  static final double     LEG_TIME_3_3   = 1;
  static final double     LEG_TIME_4_3  = 1;

  NormalizedColorSensor colorSensor;

  View relativeLayout;

  @Override public void runOpMode() {
    robot.init(hardwareMap);
    // Get a reference to the RelativeLayout so we can later change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    try {
      runSample(); // actually execute the sample
    } finally {
      // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
      // as pure white, but it's too much work to dig out what actually was used, and this is good
      // enough to at least make the screen reasonable again.
      // Set the panel back to the default color
      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.WHITE);
        }
      });
      }
  }

  protected void runSample() {

    float gain = 1;

    int path = 0;

    boolean finished = false;
    boolean check2 = false;

    final float[] hsvValues = new float[3];

    // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
    // state of the X button on the gamepad
    boolean xButtonPreviouslyPressed = false;
    boolean xButtonCurrentlyPressed = false;

    // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
    // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
    // the values you get from ColorSensor are dependent on the specific sensor you're using.
    colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

    // If possible, turn the light on in the beginning (it might already be on anyway,
    // we just make sure it is if we can).
    if (colorSensor instanceof SwitchableLight) {
      ((SwitchableLight)colorSensor).enableLight(true);
    }
    colorSensor.setGain(gain);
    // Wait for the start button to be pressed.
    waitForStart();

    double turn_left_right = 0;
    double forward_backward = 0;
    double strafe_left_right = 0;
    double servopos1 = 0.95;
    double servopos2 = 0.1;
    double Lift_Power = 0;

    servopos1 = 0.87;
    servopos2 = 0.11;

    robot.servo.setPosition(servopos1);
    robot.servo2.setPosition(servopos2);

    //drive to cone

      //short drive
    runtime.reset();

      turn_left_right = 0;
      forward_backward = 0;
      strafe_left_right = 0;
      servopos1 = 0.87;
      servopos2 = 0.11;
      Lift_Power = 0;

      robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
      robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
      robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
      robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
      robot.servo.setPosition(servopos1);
      robot.servo2.setPosition(servopos2);
      robot.lift.setPower(Lift_Power);
      while (opModeIsActive() && (runtime.seconds() < LEG_TIME_1_start_1)) {
        telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
      }

      runtime.reset();

      turn_left_right = 0;
      forward_backward = 0.255;
      strafe_left_right = 0;
      servopos1 = 0.87;
      servopos2 = 0.11;
      Lift_Power = 0.50;

      robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
      robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
      robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
      robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
      robot.servo.setPosition(servopos1);
      robot.servo2.setPosition(servopos2);
      robot.lift.setPower(Lift_Power);
      while (opModeIsActive() && (runtime.seconds() < LEG_TIME_1_start)) {
        telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
      }
      runtime.reset();
      turn_left_right = 0;
      forward_backward = 0;
      strafe_left_right = 0;
      Lift_Power = 0.25;
      robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
      robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
      robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
      robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
      robot.servo.setPosition(servopos1);
      robot.servo2.setPosition(servopos2);
      robot.lift.setPower(Lift_Power);
    //pause and check distance
    runtime.reset();
    while (runtime.seconds() < 2) {
      if (colorSensor instanceof DistanceSensor) {
        telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
      }
      telemetry.update();
    }
    // read color

    gain = 5;
    colorSensor.setGain(gain);

    //turn off light
    if (colorSensor instanceof SwitchableLight) {
      ((SwitchableLight)colorSensor).enableLight(false);
    }

    runtime.reset();
    while (runtime.seconds() < 2.5) {
      NormalizedRGBA colors = colorSensor.getNormalizedColors();

      // Update the hsvValues array by passing it to Color.colorToHSV()
      Color.colorToHSV(colors.toColor(), hsvValues);

      telemetry.addLine()
              .addData("Red", "%.3f", colors.red)
              .addData("Green", "%.3f", colors.green)
              .addData("Blue", "%.3f", colors.blue);
      telemetry.addLine()
              .addData("Hue", "%.3f", hsvValues[0])
              .addData("Saturation", "%.3f", hsvValues[1])
              .addData("Value", "%.3f", hsvValues[2]);
      telemetry.addData("Alpha", "%.3f", colors.alpha);

      telemetry.update();

        if (colors.blue <= 0.15 & colors.green <= 0.2 & colors.red <= 0.15) {
          //black 1
          path = 1;
        } else if(colors.blue >= 0.5 & colors.green >= 0.5 & colors.red >= 0.5) {
          // white 2
          path = 2;
        } else {
          //else green
          path = 3;
        }
      } //while

    runtime.reset();

    turn_left_right = 0;
    forward_backward = 0.25;
    strafe_left_right = 0;
    servopos1 = 0.87;
    servopos2 = 0.11;
    Lift_Power = 0.25;

    robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
    robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
    robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
    robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
    robot.servo.setPosition(servopos1);
    robot.servo2.setPosition(servopos2);
    robot.lift.setPower(Lift_Power);
    while (opModeIsActive() && (runtime.seconds() < LEG_TIME_2_start)) {
      telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
      telemetry.update();
    }
    runtime.reset();

    turn_left_right = 0;
    forward_backward = -0.25;
    strafe_left_right = 0;
    servopos1 = 0.87;
    servopos2 = 0.11;
    Lift_Power = 0.25;

    robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
    robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
    robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
    robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
    robot.servo.setPosition(servopos1);
    robot.servo2.setPosition(servopos2);
    robot.lift.setPower(Lift_Power);
    while (opModeIsActive() && (runtime.seconds() < LEG_TIME_3_start)) {
      telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
      telemetry.update();
    }

    runtime.reset();

    turn_left_right = 0;
    forward_backward = 0;
    strafe_left_right = 0;

    robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
    robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
    robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
    robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
    robot.servo.setPosition(servopos1);
    robot.servo2.setPosition(servopos2);
    robot.lift.setPower(Lift_Power);
    while (opModeIsActive() && (runtime.seconds() < LEG_TIME_1_score)) {
      telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
      telemetry.update();
    }
    runtime.reset();

    turn_left_right = 0.25;
    forward_backward = 0;
    strafe_left_right = 0;

    robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
    robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
    robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
    robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
    robot.servo.setPosition(servopos1);
    robot.servo2.setPosition(servopos2);
    robot.lift.setPower(Lift_Power);
    while (opModeIsActive() && (runtime.seconds() < LEG_TIME_2_score)) {
      telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
      telemetry.update();
    }
    runtime.reset();

    turn_left_right = 0;
    forward_backward = 0.35;
    strafe_left_right = 0;
    Lift_Power = 0.75;

    robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
    robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
    robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
    robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
    robot.servo.setPosition(servopos1);
    robot.servo2.setPosition(servopos2);
    robot.lift.setPower(Lift_Power);
    while (opModeIsActive() && (runtime.seconds() < LEG_TIME_3_score)) {
      telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
      telemetry.update();
    }
    runtime.reset();
    turn_left_right = -0.25;
    forward_backward = 0;
    strafe_left_right = 0;
    Lift_Power = 0.95;

    robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
    robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
    robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
    robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
    robot.servo.setPosition(servopos1);
    robot.servo2.setPosition(servopos2);
    robot.lift.setPower(Lift_Power);
    while (opModeIsActive() && (runtime.seconds() < LEG_TIME_4_score)) {
      telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
      telemetry.update();
    }
    runtime.reset();
    turn_left_right = 0;
    forward_backward = 0.35;
    strafe_left_right = 0;
    Lift_Power = 0.35;

    robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
    robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
    robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
    robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
    robot.servo.setPosition(servopos1);
    robot.servo2.setPosition(servopos2);
    robot.lift.setPower(Lift_Power);
    while (opModeIsActive() && (runtime.seconds() < LEG_TIME_5_score)) {
      telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
      telemetry.update();
    }
    runtime.reset();
    turn_left_right = 0;
    forward_backward = 0;
    strafe_left_right = 0;
    Lift_Power = 0.35;

    robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
    robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
    robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
    robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
    robot.servo.setPosition(servopos1);
    robot.servo2.setPosition(servopos2);
    robot.lift.setPower(Lift_Power);
    while (opModeIsActive() && (runtime.seconds() < LEG_TIME_6_score)) {
      telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
      telemetry.update();
    }
    runtime.reset();
    turn_left_right = 0;
    forward_backward = 0;
    strafe_left_right = 0;
    Lift_Power = 0.35;
    servopos1 = 0.95;
    servopos2 = 0.05;

    robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
    robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
    robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
    robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
    robot.servo.setPosition(servopos1);
    robot.servo2.setPosition(servopos2);
    robot.lift.setPower(Lift_Power);
    while (opModeIsActive() && (runtime.seconds() < LEG_TIME_7_score)) {
      telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
      telemetry.update();
    }
    runtime.reset();
    turn_left_right = 0;
    forward_backward = -0.35;
    strafe_left_right = 0;
    Lift_Power = 0.35;
    servopos1 = 0.95;
    servopos2 = 0.05;

    robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
    robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
    robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
    robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
    robot.servo.setPosition(servopos1);
    robot.servo2.setPosition(servopos2);
    robot.lift.setPower(Lift_Power);
    while (opModeIsActive() && (runtime.seconds() < LEG_TIME_8_score)) {
      telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
      telemetry.update();
    }
    runtime.reset();

    //if the else black,white, green

//    if ((path == 1) && opModeIsActive()) {
//      telemetry.addData("Path", "black: %2.5f S Elapsed", runtime.seconds());
//      telemetry.update();
//    }
//    else if ((path == 2) && opModeIsActive()) {
//      telemetry.addData("Path", "white: %2.5f S Elapsed", runtime.seconds());
//      telemetry.update();
//    }
//    else if ((path == 3) && opModeIsActive()) {
//      telemetry.addData("Path", "green %2.5f S Elapsed", runtime.seconds());
//      telemetry.update();
//    }
    sleep(100000);
    //black
    if ((path == 1) && opModeIsActive()) {
      runtime.reset();
      turn_left_right = 0;
      forward_backward = 0;
      strafe_left_right = 0;
      robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
      robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
      robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
      robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
      robot.servo.setPosition(servopos1);
      robot.servo2.setPosition(servopos2);
      robot.lift.setPower(Lift_Power);
      //pause and check distance
      runtime.reset();
      while (opModeIsActive() && (runtime.seconds() < LEG_TIME_1_1)) {
        telemetry.addData("Path", "Leg 1 black: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
      }

      turn_left_right = 0;
      forward_backward = 0.10;
      strafe_left_right = -0.35;

      robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
      robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
      robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
      robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
      robot.servo.setPosition(servopos1);
      robot.servo2.setPosition(servopos2);
      robot.lift.setPower(Lift_Power);
      runtime.reset();
      while (opModeIsActive() && (runtime.seconds() < LEG_TIME_2_1)) {
        telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
      }

      turn_left_right = 0;
      forward_backward = 0;
      strafe_left_right = -0.5;

      robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
      robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
      robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
      robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
      robot.servo.setPosition(servopos1);
      robot.servo2.setPosition(servopos2);
      robot.lift.setPower(Lift_Power);
      runtime.reset();
      while (opModeIsActive() && (runtime.seconds() < LEG_TIME_3_1)) {
        telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
      }

      robot.leftFrontDrive.setPower(0);
      robot.rightFrontDrive.setPower(0);
      robot.leftRearDrive.setPower(0);
      robot.rightRearDrive.setPower(0);
      finished = true;
      check2 = true;

    }
    //white
    if ((path == 2) && opModeIsActive()) {


      runtime.reset();
      while (opModeIsActive() && (runtime.seconds() < LEG_TIME_1_2)) {
        telemetry.addData("Path", "Leg 1 white: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
      }

      turn_left_right = 0;
      forward_backward = 0.25;
      strafe_left_right = 0;

      robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
      robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
      robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
      robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
      robot.servo.setPosition(servopos1);
      robot.servo2.setPosition(servopos2);
      robot.lift.setPower(Lift_Power);
      runtime.reset();
      while (opModeIsActive() && (runtime.seconds() < LEG_TIME_2_2)) {
        telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
      }

      turn_left_right = 0;
      forward_backward = 0;
      strafe_left_right = 0;

      robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
      robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
      robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
      robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
      robot.servo.setPosition(servopos1);
      robot.servo2.setPosition(servopos2);
      robot.lift.setPower(Lift_Power);
      runtime.reset();
      while (opModeIsActive() && (runtime.seconds() < LEG_TIME_2_2)) {
        telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
      }

      turn_left_right = 0;
      forward_backward = 0;
      strafe_left_right = 0;

      robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
      robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
      robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
      robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
      robot.servo.setPosition(servopos1);
      robot.servo2.setPosition(servopos2);
      robot.lift.setPower(Lift_Power);
      runtime.reset();
      while (opModeIsActive() && (runtime.seconds() < LEG_TIME_3_2)) {
        telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
      }

      robot.leftFrontDrive.setPower(0);
      robot.rightFrontDrive.setPower(0);
      robot.leftRearDrive.setPower(0);
      robot.rightRearDrive.setPower(0);
      robot.servo.setPosition(servopos1);
      robot.servo2.setPosition(servopos2);
      robot.lift.setPower(Lift_Power);
      finished = true;
      check2 = true;
    }
    //green
    if ((path == 3) && opModeIsActive()) {

      runtime.reset();
      while (opModeIsActive() && (runtime.seconds() < LEG_TIME_1_3)) {
        telemetry.addData("Path", "Leg 1 green: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
      }

      turn_left_right = 0;
      forward_backward = 0.25;
      strafe_left_right = 0.25;

      robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
      robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
      robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
      robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
      robot.servo.setPosition(servopos1);
      robot.servo2.setPosition(servopos2);
      robot.lift.setPower(Lift_Power);
      runtime.reset();
      while (opModeIsActive() && (runtime.seconds() < LEG_TIME_2_3)) {
        telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
      }

      turn_left_right = 0;
      forward_backward = 0;
      strafe_left_right = 0.5;

      robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
      robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
      robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
      robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
      robot.servo.setPosition(servopos1);
      robot.servo2.setPosition(servopos2);
      robot.lift.setPower(Lift_Power);
      runtime.reset();
      while (opModeIsActive() && (runtime.seconds() < LEG_TIME_3_3)) {
        telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
      }

      robot.leftFrontDrive.setPower(0);
      robot.rightFrontDrive.setPower(0);
      robot.leftRearDrive.setPower(0);
      robot.rightRearDrive.setPower(0);
      finished = true;
      check2 = true;
    }



    sleep(2000000);

    // Loop until we are asked to stop

      // Show the gain value via telemetry
      telemetry.addData("Gain", gain);

      // Tell the sensor our desired gain value (normally you would do this during initialization,
      // not during the loop)



      // If the button state is different than what it was, then act
      if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
        // If the button is (now) down, then toggle the light
        if (xButtonCurrentlyPressed) {
          if (colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight)colorSensor;
            light.enableLight(!light.isLightOn());
          }
        }
      }
      xButtonPreviouslyPressed = xButtonCurrentlyPressed;

      // Get the normalized colors from the sensor




      /* If this color sensor also has a distance sensor, display the measured distance.
       * Note that the reported distance is only useful at very close range, and is impacted by
       * ambient light and surface reflectivity. */


      telemetry.update();

      sleep(5000);


      //black




      // Change the Robot Controller's background color to match the color detected by the color sensor.
      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
        }
      });
//    }
  }
}
