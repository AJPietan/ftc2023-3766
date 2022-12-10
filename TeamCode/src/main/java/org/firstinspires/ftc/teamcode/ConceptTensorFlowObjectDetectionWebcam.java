/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Concept: TensorFlow Object Detection Webcam", group = "Concept")

public class ConceptTensorFlowObjectDetectionWebcam extends LinearOpMode {
    HardwarePushbotV3 robot   = new HardwarePushbotV3();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     LEG_TIME_1_start    = 0.6;

    static final double     LEG_TIME_1_1    = 0.5;
    static final double     LEG_TIME_2_1   = 1;
    static final double     LEG_TIME_3_1   = 1;
    static final double     LEG_TIME_4_1  = 1;

    static final double     LEG_TIME_1_2    = 2.5;
    static final double     LEG_TIME_2_2   = 1;
    static final double     LEG_TIME_3_2   = 1;
    static final double     LEG_TIME_4_2  = 1;

    static final double     LEG_TIME_1_3    = 0.5;
    static final double     LEG_TIME_2_3   = 0.8;
    static final double     LEG_TIME_3_3   = 1;
    static final double     LEG_TIME_4_3  = 1;

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "model_20221126_181829.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "3",
            "6",
            "7"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AZeBw/X/////AAABmZBpaQkDkEhxnKq9thzt3+w3IozryN/aD5c+jDxZrUN/sNAFVUkRlGC7ynOz601I8Xs0z9Gq65IISOs8Nz/ghrkJZLixsYNU1Av8EfjcbaydylP/JRgRsebPEwcctt0IpQw6OI3rg7UxwDz/GM+QRfgFMfqOxPBBU1j2dwbGeUlF6C3I/zHfdXs2ZYNaDeziVOQE6/93ocvbkwCacNauOqG8E4iLLoCgI8N1QhDHDUiK88ovZ+LFu+brhB/dtxCFdkQbpCjiDFA8R8SXt3z/ZRPpXBBswhvo1437b0n++8naLSb+HM/HMtdpXo50ifY81NMdapPxPyl38xpktsm3NTs6wo/FfZRHYOM4gKUI7KSZ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        boolean finished = false;
        boolean check2 = false;

        robot.init(hardwareMap);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        boolean timmer = false;
        if (opModeIsActive() && finished == false) {
            double startdone = 1;
            while (opModeIsActive() && !finished) {
                double turn_left_right = 0;
                double forward_backward = 0;
                double strafe_left_right = 0;

                if (startdone == 1) {
                    //short drive
                    runtime.reset();

                    startdone = 2;
                    turn_left_right = 0;
                    forward_backward = -0.35;
                    strafe_left_right = -0.2;

                    robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
                    robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
                    robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
                    robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);
                    while (opModeIsActive() && (runtime.seconds() < LEG_TIME_1_start)) {
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
                    //grab tfod stuff
                }
//                if (runtime.seconds() > 11) {
//                    telemetry.addData("# seconds = 11","", runtime.seconds());
//                    telemetry.update();
//                }

                runtime.reset();
                tfod.activate();
                tfod.setZoom(1.0, 16.0 / 9.0);

                while (opModeIsActive() && runtime.seconds()<5) {
                    telemetry.addData("# TFOD on","%2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                    sleep(100);
                }

                telemetry.addData("# TFOD ","DONE");
                telemetry.update();



                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    while (opModeIsActive()) {

                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {//Camera is likely able to read more than one image at a time, so when it doesn't see any images, it won't loop through any code.
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                            if (recognition.getLabel() == "3") {
                                runtime.reset();
                                while (opModeIsActive() && (runtime.seconds() < LEG_TIME_1_1)) {
                                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                                    telemetry.update();
                                }

                                turn_left_right = 0;
                                forward_backward = -0.25;
                                strafe_left_right = 0.20;

                                robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
                                robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
                                robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
                                robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);

                                runtime.reset();
                                while (opModeIsActive() && (runtime.seconds() < LEG_TIME_2_1)) {
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
                            if (recognition.getLabel() == "6") {

                                runtime.reset();
                                while (opModeIsActive() && (runtime.seconds() < LEG_TIME_1_3)) {
                                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                                    telemetry.update();
                                }

                                turn_left_right = 0;
                                forward_backward = -0.25;
                                strafe_left_right = -0.20;

                                robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
                                robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
                                robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
                                robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);

                                runtime.reset();
                                while (opModeIsActive() && (runtime.seconds() < LEG_TIME_2_3)) {
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
                            if (recognition.getLabel() == "7") {


                                runtime.reset();
                                while (opModeIsActive() && (runtime.seconds() < LEG_TIME_1_2)) {
                                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                                    telemetry.update();
                                }

                                turn_left_right = 0;
                                forward_backward = -0.25;
                                strafe_left_right = 0;

                                robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
                                robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
                                robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
                                robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);

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

                                runtime.reset();
                                while (opModeIsActive() && (runtime.seconds() < LEG_TIME_3_2)) {
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

                        }

                        //TODO: if still in while loop after 10 seconds, run default case here.

                        if (runtime.seconds() > 10 && check2 == false) {


                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < LEG_TIME_1_2)) {
                                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            turn_left_right = 0;
                            forward_backward = -0.5;
                            strafe_left_right = 0;

                            robot.leftFrontDrive.setPower(-turn_left_right + forward_backward + -strafe_left_right);
                            robot.rightFrontDrive.setPower(turn_left_right + forward_backward + -strafe_left_right);
                            robot.leftRearDrive.setPower(-turn_left_right + forward_backward + strafe_left_right);
                            robot.rightRearDrive.setPower(turn_left_right + forward_backward + strafe_left_right);

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

                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < LEG_TIME_3_2)) {
                                telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            robot.leftFrontDrive.setPower(0);
                            robot.rightFrontDrive.setPower(0);
                            robot.leftRearDrive.setPower(0);
                            robot.rightRearDrive.setPower(0);
                            finished = true;
                        }

                        telemetry.addData("Seconds", runtime.seconds());
                        telemetry.update();
                        if (runtime.seconds() > 10) {
                            timmer = true;
                        }
                        }
                    }
                }

                sleep(500000);
                telemetry.addData("Bottom of Loop","");
                telemetry.update();
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
