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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Pushbot: Auto Drive By TimeV3", group="Red")
//@Disabled
public class PushbotAutoDriveByTime_LinearV3 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbotV3 robot   = new HardwarePushbotV3();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     LEFT_FORWARD_SPEED = -0.5;
    static final double     RIGHT_FORWARD_SPEED = 0.5;
    static final double     TURN_SPEED    = 0.5;
    static final double     LEG_TIME_1    = 6;
    static final double     LEG_TIME_2    = 15;
    static final double     LEG_TIME_3    = 2;
    static final double     LEG_TIME_4    = 0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();

        robot.leftFrontDrive.setPower(-LEFT_FORWARD_SPEED);
        robot.leftRearDrive.setPower(-LEFT_FORWARD_SPEED);
        robot.rightFrontDrive.setPower(-RIGHT_FORWARD_SPEED);
        robot.rightRearDrive.setPower(-RIGHT_FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < LEG_TIME_1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < LEG_TIME_2)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.leftFrontDrive.setPower(LEFT_FORWARD_SPEED);
        robot.leftRearDrive.setPower(LEFT_FORWARD_SPEED);
        robot.rightFrontDrive.setPower(RIGHT_FORWARD_SPEED);
        robot.rightRearDrive.setPower(RIGHT_FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < LEG_TIME_3)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.leftFrontDrive.setPower(-LEFT_FORWARD_SPEED);
        robot.leftRearDrive.setPower(LEFT_FORWARD_SPEED);
        robot.rightFrontDrive.setPower(-RIGHT_FORWARD_SPEED);
        robot.rightRearDrive.setPower(RIGHT_FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < LEG_TIME_4)) {
            telemetry.addData("Path", "Leg 4: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightRearDrive.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
