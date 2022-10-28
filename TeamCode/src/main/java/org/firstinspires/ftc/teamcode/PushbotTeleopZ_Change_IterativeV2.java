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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Pushbot: Teleop Z Change V2 (Use this for now.)", group="Pushbot")
//@Disabled
public class PushbotTeleopZ_Change_IterativeV2 extends OpMode{

    HardwarePushbotV3 robot       = new HardwarePushbotV3();
    //Define Start Variables
//    double CurrentServoPos2 = .5;
    int profile = 0;
    int base_profile = profile + 1;
    int profile_count = 2;
    int controller_safe_zone = 10;
    int profile_switch_debug = 0;
    int bumper_check = 0;
    @Override
    public void init() {
        robot.init(hardwareMap);


    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        //Reset Variables
        double left;
        double right;

        //Define continue when button Y is pressed
        if ((gamepad1.y == true) & (profile_switch_debug == 0)) {
            profile_switch_debug = 1;
            profile = profile + 1;
            profile = (profile % profile_count);

        }
        //Define Do not continue until button Y is not pressed
        if ((profile_switch_debug == 1) & (gamepad1.y == false)) {
            profile_switch_debug = 0;
        }

        //Display Stats
        int base_profile = profile + 1;
        if (base_profile == 1) {
            telemetry.addData("Profile str", "Tank Drive");
        }
        else if (base_profile == 2) {
            telemetry.addData("Profile str", "Spider Drive");
        }

        telemetry.addData("Profile int", base_profile);
        telemetry.addData("safe zone", controller_safe_zone);

//        //Set Profile
//        if (base_profile == 1)

        //Define Movement/Controls
        if (base_profile == 1) {

            //Define Tank Drive
            left = gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            if (gamepad1.left_trigger > 0.1) {

                //Define Strafe Left
                robot.leftFrontDrive.setPower(gamepad1.left_trigger);
                robot.rightFrontDrive.setPower(gamepad1.left_trigger);
                robot.leftRearDrive.setPower(-gamepad1.left_trigger);
                robot.rightRearDrive.setPower(-gamepad1.left_trigger);

            } else if (gamepad1.right_trigger > 0.1) {

                //Define Strafe Right
                robot.leftFrontDrive.setPower(-gamepad1.right_trigger);
                robot.rightFrontDrive.setPower(-gamepad1.right_trigger);
                robot.leftRearDrive.setPower(gamepad1.right_trigger);
                robot.rightRearDrive.setPower(gamepad1.right_trigger);

            } else {

                //Define Forward Backward
                robot.leftFrontDrive.setPower(-left);
                robot.rightFrontDrive.setPower(right);
                robot.leftRearDrive.setPower(-left);
                robot.rightRearDrive.setPower(right);

            }
        }

        else if (base_profile == 2) {

            //Define Spider Drive
            double forward_backward;
            double left_right;
            double turn_right_left;

            forward_backward = -(gamepad1.left_stick_y);
            left_right = (gamepad1.left_stick_x);
            turn_right_left = -(gamepad1.right_stick_x);
            robot.leftFrontDrive.setPower(-turn_right_left + forward_backward + -left_right);
            robot.rightFrontDrive.setPower(turn_right_left + forward_backward + -left_right);
            robot.leftRearDrive.setPower(-turn_right_left + forward_backward + left_right);
            robot.rightRearDrive.setPower(turn_right_left + forward_backward + left_right);
//            //Define Forward Backward Controls
//
//            if ((gamepad1.left_stick_y > (controller_safe_zone / 100)) || (gamepad1.left_stick_y < (controller_safe_zone / -100))) {
//                forward_backward = -(gamepad1.left_stick_y);
//            }
//
//            //Define Strafe Left Right Controls
//            if ((gamepad1.left_stick_x > (controller_safe_zone / 100)) || (gamepad1.left_stick_x < (controller_safe_zone / -100))) {
//                left_right = -(gamepad1.left_stick_x);
//            }
//
//            //Define Turn Left Right Controls
//            if ((gamepad1.right_stick_x > (controller_safe_zone / 100)) || (gamepad1.right_stick_x < (controller_safe_zone / -100))) {
//                turn_right_left = -(gamepad1.right_stick_x);
//            }
//
//            //Define Camera Look Up Down
//            if ((gamepad1.right_stick_y > (controller_safe_zone / 100)) || (gamepad1.right_stick_y < (controller_safe_zone / -100))) {
//                look_up_down = -(gamepad1.right_stick_y);
//            }
//
//            //Define Change Safe Zone
//            if ((gamepad1.right_bumper == true) || (gamepad1.left_bumper == true)) {
//                if ((gamepad1.right_bumper == true) & (bumper_check == 0) & (controller_safe_zone < 100)) {
//                    controller_safe_zone = controller_safe_zone + 5;
//                    bumper_check = 1;
////                    telemetry.addData("Safe Zone = ", controller_safe_zone, "%");
//                }
//                if ((gamepad1.left_bumper == true) & (bumper_check == 0) & (controller_safe_zone > 0)) {
//                    controller_safe_zone = controller_safe_zone - 5;
//                    bumper_check = 1;
////                    telemetry.addData("Safe Zone = ", controller_safe_zone, "%");
//                }
//            }
//            else {
//                bumper_check = 0;
//            }
//
//            //Define Forward Backward
//            robot.leftFrontDrive.setPower(-forward_backward);
//            robot.rightFrontDrive.setPower(forward_backward);
//            robot.leftRearDrive.setPower(-forward_backward);
//            robot.rightRearDrive.setPower(forward_backward);
//
//            //Define Strafe Left Right
//            robot.leftFrontDrive.setPower(-left_right);
//            robot.rightFrontDrive.setPower(left_right);
//            robot.leftRearDrive.setPower(left_right);
//            robot.rightRearDrive.setPower(-left_right);
//
//            //Define Turn Left Right
//            robot.leftFrontDrive.setPower(turn_right_left);
//            robot.rightFrontDrive.setPower(turn_right_left);
//            robot.leftRearDrive.setPower(turn_right_left);
//            robot.rightRearDrive.setPower(turn_right_left);

            }
//        else if (base_profile == 3) {
//
//            //Define D-Pad Drive
//            forward_backward = 0;
//            left_right = 0;
//            turn_right_left = 0;
////            if (gamepad1.)
//            //Define Turn Left Right
//            robot.leftFrontDrive.setPower(turn_right_left);
//            robot.rightFrontDrive.setPower(turn_right_left);
//            robot.leftRearDrive.setPower(turn_right_left);
//            robot.rightRearDrive.setPower(turn_right_left);
//            }

        }

    //            if ((gamepad1.left_stick_y > (controller_safe_zone / 100)) || (gamepad1.left_stick_y < (controller_safe_zone / -100)) & (gamepad1.left_stick_x > (controller_safe_zone / 100)) || (gamepad1.left_stick_x < (controller_safe_zone / -100))) {
//                    //Define Forward Backward
//                    robot.leftFrontDrive.setPower(((-forward_backward) + (-left_right)) / 2);
//                    robot.rightFrontDrive.setPower(((forward_backward) + (left_right)) / 2);
//                    robot.leftRearDrive.setPower(((-forward_backward) + (left_right)) / 2);
//                    robot.rightRearDrive.setPower(((forward_backward) + (-left_right)) / 2);
//                }
//            else {
//                //Define Forward Backward
//                robot.leftFrontDrive.setPower(-forward_backward);
//                robot.rightFrontDrive.setPower(forward_backward);
//                robot.leftRearDrive.setPower(-forward_backward);
//                robot.rightRearDrive.setPower(forward_backward);
//
//                //Define Strafe Left Right
//                robot.leftFrontDrive.setPower(-left_right);
//                robot.rightFrontDrive.setPower(left_right);
//                robot.leftRearDrive.setPower(left_right);
//                robot.rightRearDrive.setPower(-left_right);
//
//                //Define Turn Left Right
//                robot.leftFrontDrive.setPower(turn_right_left);
//                robot.rightFrontDrive.setPower(turn_right_left);
//                robot.leftRearDrive.setPower(turn_right_left);
//                robot.rightRearDrive.setPower(turn_right_left);
//                }
//            }

//        if (gamepad2.right_trigger > .1) {
//            robot.TrackMotor.setPower(gamepad2.right_trigger);
//        }
//        else {
//            robot.TrackMotor.setPower(0);
//        }
//
//        robot.SlideMotor.setPower(SlidePower / 2);
//
//        robot.Walble_Motor.setPower(gamepad2.right_stick_y / 2);
//
//        CurrentServoPos = robot.servo2.getPosition();
//
//        if (gamepad2.dpad_down) {
//            robot.servo.setPosition(.005);
//        }
//        else {
//            robot.servo.setPosition(.5);
//        }
//
//        robot.servo2.setPosition(CurrentServoPos);
//        CurrentServoPos = robot.servo.getPosition();
//
//        robot.servo.setPosition(.99);
////        if (gamepad2.right_stick_y > 0.05 && CurrentServoPos < 1.0) {
////            robot.servo.setPosition(CurrentServoPos + 0.05);
////        }
////        else if (gamepad2.right_stick_y < -0.05 && CurrentServoPos > 0.0) {
////            robot.servo.setPosition(CurrentServoPos + -0.05);
////        }

//        if (gamepad2.a) {
//            CurrentServoPos2 = .5;
//        }
//        else if (gamepad2.b) {
//            CurrentServoPos2 = .9;
//        }

//

//        telemetry.addData("Say", "Everything is fine just fine.");
//        telemetry.addData("left",  "%.2f", left);
//        telemetry.addData("right", "%.2f", right);
//        telemetry.addData("slide", "%.2f", SlidePower);
//        telemetry.addData("CurrentServoPos","%.2f", CurrentServoPos);

    @Override
    public void stop() {
    }
}