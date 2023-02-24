
package org.firstinspires.ftc.teamcode;
//change

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class RightSideIMG extends LinearOpMode {
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
    double globalAngle, power = 1, correction;//537 is one complete rotation 258
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotor left_lift = null, right_lift =null;
    private Servo tweezers = null;
    private CRServo flagS = null;
    //private CRServo cameraC = null;
    private ColorSensor  CCsensor;
    private ColorSensor  ConeSensor;
    private TouchSensor touch;


    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "TopLeft");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "BottomLeft");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "TopRight");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "BottomRight");
        left_lift = hardwareMap.get(DcMotor.class, "Llift");
        right_lift = hardwareMap.get(DcMotor.class, "Rlift");
        tweezers = hardwareMap.get(Servo.class, "tweezers");
        CCsensor = hardwareMap.get(ColorSensor.class,"ccsensor");
        ConeSensor = hardwareMap.get(ColorSensor.class,"ConeSensor");
        touch = hardwareMap.get(TouchSensor.class, "touch");

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

        // SENSOR VALUES
        //Always check the values, Values can change depending on the brightness of the room

        int BlueCC = 350, RedCC = 250; //Camera color sensor (hilhi upstairs : blue = 460 and red = 340 )
        int BlueFC =200, RedFC = 200;//sensor on the salon door means (Front Color sensor)
        int BlueCone = 225, RedCone = 250;//oposite to the camera color sensor

        // SENSOR VALUES

        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == MIDDLE) {
            power = 1;

            high();
            sleep(2900);

            sright();
            sleep(80);

            tright();
            sleep(450);

            rest();

            power = 0.45;
            backwards();
            sleep(500);

            resetAngle();

            rest();
            sleep(100);

            while (opModeIsActive() && (CCsensor.red() < RedCC && CCsensor.blue() < BlueCC)) {
                power = 0.5;//100
                sleft();
            }// Cone sensor is the sensor on the left side of the robot just havent changed it yet

            power = 1;
            rest();
            sleep(100);

            sright();
            sleep(120);

            power = 0.7;
            backwards();
            sleep(100);

            rest();
            sleep(100);


            resetAngle();
            straight();
            sleep(658);

            rest();
            sleep(100);

            droping();
            sleep(1700);

            rest();
            sleep(100);

            power = 1;

            backwards();
            sleep(5);

            rest();
            sleep(100);

            power = 0.8;

            sright();
            sleep(415);

            rest();
            sleep(150);

            resetAngle();
            straight();
            sleep(475);

            rest();
            sleep(150);

            rotate(72,0.9);
            sleep(600);

            rest();
            sleep(200);

            resetAngle();
            straight();
            sleep(300);

            rest();
            sleep(200);
/*
            while (opModeIsActive() && (touch.isPressed() == false)){
                power = 0.8;
                straight();

            }*/
            //scores 1 and parks in middle
        }
        /*
         * Handle LEFT 1
         */
        if (tagOfInterest.id == LEFT) {
            power = 1;

            high();
            sleep(2900);

            sright();
            sleep(80);

            tright();
            sleep(450);

            rest();

            power = 0.45;
            backwards();
            sleep(500);

            resetAngle();

            rest();
            sleep(100);

            while (opModeIsActive() && (CCsensor.red() < RedCC && CCsensor.blue() < BlueCC)) {
                power = 0.5;//100
                sleft();
            }// Cone sensor is the sensor on the left side of the robot just havent changed it yet

            power = 1;
            rest();
            sleep(100);

            sright();
            sleep(120);

            power = 0.7;
            backwards();
            sleep(100);

            rest();
            sleep(100);


            resetAngle();
            straight();
            sleep(658);

            rest();
            sleep(100);

            droping();
            sleep(1700);

            rest();
            sleep(100);

            power = 1;

            backwards();
            sleep(5);

            rest();
            sleep(100);

            power = 0.8;

            sright();
            sleep(415);

            rest();
            sleep(150);

            resetAngle();
            straight();
            sleep(475);

            rest();
            sleep(150);

            rotate(72,0.9);
            sleep(600);

            rest();
            sleep(200);
        }
        /*
         * Handle RIGHT 3
         */
        if (tagOfInterest.id == RIGHT) {//tagOfInterest.id
            power = 1;

            high();
            sleep(2900);

            sright();
            sleep(80);

            tright();
            sleep(450);

            rest();

            power = 0.45;
            backwards();
            sleep(500);

            resetAngle();

            rest();
            sleep(100);

            while (opModeIsActive() && (CCsensor.red() < RedCC && CCsensor.blue() < BlueCC)) {
                power = 0.5;//100
                sleft();
            }// Cone sensor is the sensor on the left side of the robot just havent changed it yet

            power = 1;
            rest();
            sleep(100);

            sright();
            sleep(120);

            power = 0.7;
            backwards();
            sleep(100);

            rest();
            sleep(100);


            resetAngle();
            straight();
            sleep(658);

            rest();
            sleep(100);

            droping();
            sleep(1700);

            rest();
            sleep(100);

            power = 1;

            backwards();
            sleep(5);

            rest();
            sleep(100);

            power = 1;

            sright();
            sleep(1415);

            rest();
            sleep(100);

            resetAngle();
            straight();
            sleep(200);

            rest();
            sleep(200);
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
        correction = checkDirection();

        // set power levels.
        telemetry.addData("ANGLE:", getAngle());
        telemetry.update();

        leftFrontDrive.setPower(power - correction);
        rightFrontDrive.setPower(power + correction);//goes STRAIGHT
        leftBackDrive.setPower(power - correction);
        rightBackDrive.setPower(power + correction);
        sleep(100);
    }

    public void backwards () {
        correction = checkDirection();

        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(-power );//goes backwards
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(-power);
        sleep(100);
    }
    public void tright () {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);
    }
    public void tleft () {
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
        sleep(100);
    }
    public void sright () {
        correction = checkDirection();

        // set power levels.
        telemetry.addData("ANGLE:", getAngle());
        telemetry.update();


        leftFrontDrive.setPower(power + correction);
        rightFrontDrive.setPower(-power - correction);
        leftBackDrive.setPower(-power + correction);
        rightBackDrive.setPower(power - correction);
        sleep(100);
    }
    public void sleft () {
        correction = checkDirection();

        // set power levels.
        telemetry.addData("ANGLE:", getAngle());
        telemetry.update();

        leftFrontDrive.setPower(-power - correction);//correction slows it down or speeds it up
        rightFrontDrive.setPower(power + correction);
        leftBackDrive.setPower(power - correction);
        rightBackDrive.setPower(-power + correction);//was -
        sleep(50);
    }

    public void diagLeft () {
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        //leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        // rightBackDrive.setPower(power);
        sleep(50);
    }
    public void diagRight () {
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftFrontDrive.setPower(power);

        //rightFrontDrive.setPower(power);
        //leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep(50);
    }


    public void rest () {
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);//BRAKE
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
        sleep(100);
    }
    public void grabing(){
        tweezers.setPosition(0);
    }
    public void droping (){
        tweezers.setPosition(1);
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

    public void low(){

        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_lift.setTargetPosition(-1200);
        left_lift.setTargetPosition(-1200);

        right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        left_lift.setPower(0.8);
        right_lift.setPower(0.8);

    }

    public void ToHigh(){

        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_lift.setTargetPosition(-2470);
        left_lift.setTargetPosition(-2470);

        right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        left_lift.setPower(1);
        right_lift.setPower(1);

    }

    public void high(){
        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_lift.setTargetPosition(-2870);
        left_lift.setTargetPosition(-2870);


        right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        left_lift.setPower(1);
        right_lift.setPower(1);

    }

    public  void lowerTC(){
        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_lift.setTargetPosition(770);
        left_lift.setTargetPosition(770);


        right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        left_lift.setPower(0.8);
        right_lift.setPower(0.8);

    }
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .045;

        angle = getAngle();

        if (angle == 0)
            correction = 0;// no adjustment.

        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
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
        telemetry.addData("BLUE Csensor:", CCsensor.blue());
        telemetry.addData("RED Csensor:", CCsensor.red());
        telemetry.addLine();
        telemetry.addData("BLUE Conesensor:", ConeSensor.blue());
        telemetry.addData("RED Conesensor:", ConeSensor.red());
    }
}