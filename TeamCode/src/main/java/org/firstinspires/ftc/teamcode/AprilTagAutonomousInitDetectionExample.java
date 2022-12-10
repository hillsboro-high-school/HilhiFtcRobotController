/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    // Our camera is Logitech c920
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.045;

    // THree tags from the 36h11 family
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    AprilTagDetection tagOfInterest = null;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode()
    {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "TopLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BottomLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "TopRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BottomRight");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }// DEBUG was 800x448

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        waitForStart();
        //go left a bit
        leftFrontDrive.setPower(-.5);//LEFT CODE
        rightFrontDrive.setPower(.5);
        leftBackDrive.setPower(.5);
        rightBackDrive.setPower(-.5);

        sleep(250);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        sleep(1000);

        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime > 5000) { // time to check for in ms
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    // telemetry.addData("Tag:", tag.id);
                    // telemetry.update();

                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        //telemetry.addData("Tag:", tag.id);
                        //telemetry.update();
                        sleep(1000);
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        if(tagOfInterest != null)
        {
            if (tagOfInterest.id == MIDDLE || tagOfInterest.id == LEFT || tagOfInterest.id == RIGHT) {
                leftFrontDrive.setPower(0.5);//RIGHT CODE
                rightFrontDrive.setPower(-0.5);
                leftBackDrive.setPower(-0.5);
                rightBackDrive.setPower(0.5);

                sleep(250);

                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
            }
            /* Actually do something useful */
            if(tagOfInterest.id == MIDDLE)
            {
                /*
                 * Do MIDDLE code because tag was not seen so why not guess? 2
                 */
                sleep(500);

                leftFrontDrive.setPower(.5);
                rightFrontDrive.setPower(.5);
                leftBackDrive.setPower(.5);
                rightBackDrive.setPower(.5);

                sleep(1000);

                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);


            }
            else
            {
                /*
                 * Handle LEFT 1
                 */
                if (tagOfInterest.id == LEFT) {

                    sleep(500);

                    leftFrontDrive.setPower(.5);
                    rightFrontDrive.setPower(.5);
                    leftBackDrive.setPower(.5);
                    rightBackDrive.setPower(.5);

                    sleep(1000);

                    leftFrontDrive.setPower(-.5);//LEFT CODE
                    rightFrontDrive.setPower(.5);
                    leftBackDrive.setPower(.5);
                    rightBackDrive.setPower(-.5);

                    sleep(1350);

                    leftFrontDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightBackDrive.setPower(0);
                }
                /*
                 * Handle RIGHT 3
                 */
                else {

                    sleep(500);

                    leftFrontDrive.setPower(.5);
                    rightFrontDrive.setPower(.5);
                    leftBackDrive.setPower(.5);
                    rightBackDrive.setPower(.5);

                    sleep(1000);

                    leftFrontDrive.setPower(0.5);//RIGHT CODE
                    rightFrontDrive.setPower(-0.5);
                    leftBackDrive.setPower(-0.5);
                    rightBackDrive.setPower(0.5);

                    sleep(1350);

                    leftFrontDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightBackDrive.setPower(0);
                }

            }
        } else {
            // didnt see tag
            leftFrontDrive.setPower(0.5);//RIGHT CODE
            rightFrontDrive.setPower(-0.5);
            leftBackDrive.setPower(-0.5);
            rightBackDrive.setPower(0.5);

            sleep(250);

            leftFrontDrive.setPower(.5);
            rightFrontDrive.setPower(.5);
            leftBackDrive.setPower(.5);
            rightBackDrive.setPower(.5);
        }

        // by now we may or may not have seen the tag, do whatever else you need

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}

