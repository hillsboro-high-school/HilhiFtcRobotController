
package org.firstinspires.ftc.teamcode;
//change
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


import java.util.ArrayList;

@Autonomous
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.025;

    // THree tags from the 36h11 family
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    AprilTagDetection tagOfInterest = null;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = 0.30;
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotor left_lift = null, right_lift =null;
    private CRServo tweezers = null;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "TopLeft");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "BottomLeft");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "TopRight");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "BottomRight");
        left_lift = hardwareMap.get(DcMotor.class, "Llift");
        right_lift = hardwareMap.get(DcMotor.class, "Rlift");
        tweezers = hardwareMap.get(CRServo.class, "tweezers");

        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);


        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    // telemetry.addData("Tag:", tag.id);
                    // telemetry.update();

                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        telemetry.addData("Tag:", tag.id + 1);
                        // telemetry.update();
                        //
                        // sleep(1000);
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
//1
                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");
                }

            }

            telemetry.update();
            sleep(20);
        }

        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        waitForStart();
        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == MIDDLE) {

            grabing();
            sleep(1400);

            //medium();
            high();
            sleep(3500);

            sright();
            sleep(150);

            //power = 0.3
            rotate(86, power);//still requires a sleep
            sleep(400);

            rest();

            straight();
            sleep(3200);//3100

            rest();

            sleft();
            sleep(1775);

            rest();


            straight();
            sleep(430);//was 700

            rest();
            sleep (100);

            tright();//left and right are oppisite rn
            sleep(390);

            rest();

            straight();
            sleep(100);

            rest();

            droping();
            sleep(1000);

            rest();
            sleep(150);

            tleft();
            sleep(390);

            rest();

            backwards();
            sleep(400);

            rest();
            sleep(150);

            sright();
            sleep(1500);

            rest();
            sleep(150);

            rotate(84,power);
            sleep(350);

            rest();
            sleep(100);

            straight();
            sleep(50);
        }
        /*
         * Handle LEFT 1
         */
        if (tagOfInterest.id == LEFT) {

            grabing();
            sleep(1400);

            //medium();
            high();
            sleep(3500);

            sright();
            sleep(150);

            //power = 0.3
            rotate(86, power);//still requires a sleep
            sleep(400);

            rest();

            straight();
            sleep(3200);//3100

            rest();

            sleft();
            sleep(1775);

            rest();

            straight();
            sleep(430);//was 700

            rest();
            sleep (100);

            tright();//left and right are oppisite rn
            sleep(390);

            rest();

            straight();
            sleep(100);

            rest();

            droping();
            sleep(1000);

            rest();
            sleep(150);

            tleft();
            sleep(390);

            rest();

            backwards();
            sleep(400);

            rest();
            sleep(150);

            sleft();
            sleep(1570);

            rest();

            backwards();
            sleep(150);

            rest();



        }
        /*
         * Handle RIGHT 3
         */
        if (tagOfInterest.id == RIGHT) {//tagOfInterest.id
            grabing();
            sleep(1400);

            //medium();
            high();
            sleep(3500);

            sright();
            sleep(150);

            //power = 0.3
            rotate(86, power);//still requires a sleep
            sleep(400);

            rest();

            straight();
            sleep(3200);//3100

            rest();

            sleft();
            sleep(1775);

            rest();

            straight();
            sleep(430);//was 700

            rest();
            sleep (100);

            tright();//left and right are oppisite rn
            sleep(390);

            rest();

            straight();
            sleep(100);

            rest();

            droping();
            sleep(1000);

            rest();

            backwards();
            sleep(250);

            rest();

            sright();
            sleep(1850);

            rotate(91, power);//still requires a sleep
            sleep(400);

            rest();

            straight();
            sleep(2000);

            rest();

            coneS();
            sleep(3000);

            rest();
        }


    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void rotate(int degrees, double power) {
        double LFPower, RFPower, LBPower, RBPower;
        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)//turn left
        {
            LFPower = -power;
            RFPower = power;
            LBPower = -power;
            RBPower = power;
        } else if (degrees > 0)//turn right
        {
            LFPower = power;
            RFPower = -power;
            LBPower = power;
            RBPower = -power;
        } else return;

        leftFrontDrive.setPower(LFPower);
        rightFrontDrive.setPower(RFPower);
        leftBackDrive.setPower(LBPower);
        rightBackDrive.setPower(RBPower);
        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() < -degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && (getAngle() > -degrees)){
                telemetry.addData("Angle:", getAngle());
                telemetry.update();
            }

        // turn the motors off.
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }


    public void straight () {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);//goes STRAIGHT
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep(100);
    }

    public void backwards () {
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(-power);//goes backwards
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(-power);
        sleep(100);
    }
    public void tleft () {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);
    }
    public void tright () {
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
        sleep(100);
    }
    public void sright () {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
        sleep(100);
    }
    public void sleft () {
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);
        sleep(100);
    }
    public void rest () {
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
        sleep(100);
    }
    public void grabing(){
        tweezers.setPower(180);
    }
    public void droping (){
        tweezers.setPower(-180);
    }
    public void coneS (){
        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_lift.setTargetPosition(2000);
        left_lift.setTargetPosition(2000);

        right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        left_lift.setPower(0.8);
        right_lift.setPower(0.8);

    }
    public void medium(){

        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_lift.setTargetPosition(-2025);
        left_lift.setTargetPosition(-2025);

        right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        left_lift.setPower(0.8);
        right_lift.setPower(0.8);

    }
    public void high(){
        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_lift.setTargetPosition(-2823);
        left_lift.setTargetPosition(-2823);


        right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        left_lift.setPower(0.8);
        right_lift.setPower(0.8);

    }


    void tagToTelemetry (AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}



