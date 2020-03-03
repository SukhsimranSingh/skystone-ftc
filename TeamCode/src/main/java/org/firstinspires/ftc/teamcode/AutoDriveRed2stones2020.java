package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Vector;

import static java.lang.Enum.valueOf;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoDriveRed2stones2020 extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorSimple liftR;
    private DcMotorSimple liftL;
    private ServoImplEx SSGrabber;
    private CRServo grabber;
    private DistanceSensor wallSensorRed;
    private OpenCvCamera webcamRed;
    private ImprovedSkystoneDetector skyStoneDetector;
    DigitalChannel bottomSwitch;
    int SS;
    SukhHardware drive;
    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 16.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    MecanumConstraints constraints = new MecanumConstraints(
            new DriveConstraints(70.0, 60.0, 0.0, Math.toRadians(180.0), Math.toRadians(180.0), 0.0),
            18.9, 14.0);


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SukhHardware(hardwareMap);


        wallSensorRed = hardwareMap.get(DistanceSensor.class, "wallSensorRed");
        liftR = hardwareMap.get(DcMotorSimple.class, "liftR");
        liftL = hardwareMap.get(DcMotorSimple.class, "liftL");
        grabber = hardwareMap.get(CRServo.class, "grabber");
        bottomSwitch = hardwareMap.get(DigitalChannel.class, "bottomSwitch");
//        SSGrabber =  hardwareMap.get(ServoImplEx.class, "SSGrabber");

        // LP = negative moves lift up
        // LP = positive moves lift down

//        SSGrabber.setPwmRange(new PwmControl.PwmRange(750, 2250));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamRed = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "webcamRed"), cameraMonitorViewId);

        webcamRed.openCameraDevice();

        skyStoneDetector = new ImprovedSkystoneDetector();
        webcamRed.setPipeline(skyStoneDetector);

        webcamRed.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        /**
         * current autonomous**/
//        splineTest();
//        foundation();
//        runtest();
          gyroDrive(.6,10,0);
          gyroTurn(.6,90);

//        splineTestFull();
//        foundationSpline();
//        run();

        //TODO jump
    }


    public void PIDforward(double Distance) {

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(Distance)
                .build();

        if (opModeIsActive()) {

            if (isStopRequested()) return;

            drive.followTrajectorySync(trajectory);
        }
    }

    public void PIDTurn(double angle) {

        if (opModeIsActive()) {
            drive.turnSync(Math.toRadians(angle));
        }
    }

    public void PIDwalk() {
        if (opModeIsActive()) return;

        while (opModeIsActive()) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(48)
                            .build()
            );
        }
    }

    public void PIDstrafeLeft(double Distance) {

        Trajectory trajectory = drive.trajectoryBuilder()
                .strafeLeft(Distance)
                .build();

        if (opModeIsActive()) {

            if (isStopRequested()) return;

            drive.followTrajectorySync(trajectory);
        }
    }

    public void PIDstrafeRight(double Distance) {

        Trajectory trajectory = drive.trajectoryBuilder()
                .reverse()
                .strafeRight(Distance)
                .build();

        if (opModeIsActive()) {

            if (isStopRequested()) return;

            drive.followTrajectorySync(trajectory);
        }
    }

    public void PIDback(double Distance) {

        Trajectory trajectory = drive.trajectoryBuilder()
                .back(Distance)
                .build();

        if (opModeIsActive()) {

            if (isStopRequested()) return;

            drive.followTrajectorySync(trajectory);
        }
    }

    public void getBlock() {
        grabber.setPower(-.75);
        double startTime = runtime.seconds();
        while (startTime + 1 >= runtime.seconds() && opModeIsActive()) {
            //do nothing
        }
        grabber.setPower(0);


    }

    public void liftPower(double lP, double time) {
        double startTime = runtime.seconds();
        if (opModeIsActive()) {

            if (bottomSwitch.getState() == true) {
                telemetry.addData("limit switch", "Is not Pressed");
            } else if (bottomSwitch.getState() == false) {
                lP = Range.clip(lP, -1, 0); //motors are reversed
                telemetry.addData("limit switch", "Is pressed");
            }
            liftL.setPower(lP);
            liftR.setPower(-lP);
            while (startTime + time >= runtime.seconds()) {
                //do nothing
            }
            liftL.setPower(0);
            liftR.setPower(0);
        }
    }

    public void grabBlock() {
        grabber.setPower(-.75);
        double Time = runtime.seconds();
        while (Time + 2.1 >= runtime.seconds() && opModeIsActive()) {
            //do nothing
        }
        grabber.setPower(0);
    }
    public void splineTestFull() {
        if (opModeIsActive()) {
            double startTime = runtime.seconds();
            while (startTime + .4 >= runtime.seconds() && opModeIsActive()) {
                sleep(250);
                double blockPos = skyStoneDetector.getScreenPosition().x;
                telemetry.addData("Block Pos", blockPos);
                if (blockPos <= 160) {
                    SS = 1;
                    //left
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(35, 25, Math.toRadians(0)))
                                    .build()
                    );
                    PIDforward(2);
                    sleep(100);
                    grabBlock();
                    PIDback(4);
                    telemetry.addData("block is", "left");
                    telemetry.update();
                } else if (blockPos >= 320) {
                    SS = 0;
                    //right
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(34, 13, Math.toRadians(0)))
                                    .build()
                    );
                    sleep(100);
                    grabBlock();
                    PIDback(4);

                    telemetry.addData("block is", "right");
                    telemetry.update();
                } else {
                    SS = -1;
                    //middle
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(35, 25, Math.toRadians(0)))
                                    .build()
                    );
                    sleep(100);
                    PIDforward(2);
                    grabBlock();
                    PIDback(4);

                    telemetry.addData("block is", "middle");
                    telemetry.update();

                }
            }
        }
    }
    public void liftMovement(double lP) {
        if (opModeIsActive()) {

            if (bottomSwitch.getState() == true) {
                telemetry.addData("limit switch", "Is not Pressed");
            } else if (bottomSwitch.getState() == false) {
                lP = Range.clip(lP, -1, 0); //motors are reversed
                telemetry.addData("limit switch", "Is pressed");
            }
            liftL.setPower(lP);
            liftR.setPower(-lP);

        }
    }
    public void foundationSpline() {
        if (opModeIsActive()) {
            double x = drive.getPoseEstimate().getX();
            double y = drive.getPoseEstimate().getY();
            double H = drive.getPoseEstimate().getHeading();
            switch(SS) {
                case 1:
                    //left
                    foundationLeft();
                    break;
                case 0:
                    //right
                    foundationRight();
                    break;
                case -1:
                    //middle
                    foundationMiddle();
                    break;
            }

        }
    }
    public void strafeDistanceSensor() {
        while (wallSensorRed.getDistance(DistanceUnit.INCH) < 20.5 && opModeIsActive()) {
            drive.setMotorPowers(.2, -.2, .2, -.2);
        }

    }
    public void foundationLeft() {
        if (opModeIsActive()) {
            sleep(100);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse()
                            .splineTo(new Pose2d(20, 40, Math.toRadians(-90)))
                            .build()
            );
            sleep(100);
            strafeDistanceSensor();
            sleep(250);
            PIDTurn(-10);
            sleep(100);
            double x = drive.getPoseEstimate().getX();
            double y = drive.getPoseEstimate().getY();
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(x, -60))
                            .splineTo(new Pose2d(x, -75, Math.toRadians(0)))
                            .build()
            );
            sleep(100);
            double startTimeDrive = runtime.seconds();
            drive.setMotorPowers(.2, .2, .2, .2);
            while (startTimeDrive + .8 >= runtime.seconds() && opModeIsActive()) {
                liftMovement(-.5);
            }
            drive.setMotorPowers(0, 0, 0, 0);
            liftMovement(0);
            dropBlock();
            PIDforward(2);
            liftPower(.7, .5);
           PIDback(56);
        }
    }
    public void dropBlock() {
        if (opModeIsActive()) {
            grabber.setPower(.75);

            double startTime = runtime.seconds();
            while (startTime + 1 >= runtime.seconds() && opModeIsActive()) {
                //do nothing
            }
            grabber.setPower(0);

        }
    }
    public void foundationRight() {
        double x = drive.getPoseEstimate().getX();
        double y = drive.getPoseEstimate().getY();
        double H = drive.getPoseEstimate().getHeading();
        //right
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(new Vector2d(x - 7, y))
                        .splineTo(new Pose2d(34, -30, Math.toRadians(90)))
                        .build()
        );
        sleep(100);
        PIDTurn(-30);
        sleep(100);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(new Vector2d(34, -80))
                        .splineTo(new Pose2d(24, -84, Math.toRadians(0)))
                        .build()
        );
        sleep(100);
        double startTimeDrive = runtime.seconds();
        drive.setMotorPowers(.3, .3, .3, .3);
        while (startTimeDrive + 1 >= runtime.seconds() && opModeIsActive()) {
            liftMovement(-.4);
        }
        drive.setMotorPowers(0, 0, 0, 0);
        liftMovement(0);
        dropBlock();
        PIDforward(2);
        liftPower(.7, .5);
        PIDback(56);

    }
    public void foundationMiddle() {

        sleep(100);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(20, 40, Math.toRadians(-90)))
                        .build()
        );
        sleep(100);
        strafeDistanceSensor();
        sleep(250);
        PIDTurn(-10);
        sleep(100);
        double x = drive.getPoseEstimate().getX();
        double y = drive.getPoseEstimate().getY();
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(x, -60))
                        .splineTo(new Pose2d(x, -75, Math.toRadians(0)))
                        .build()
        );
        sleep(100);
        double startTimeDrive = runtime.seconds();
        drive.setMotorPowers(.2, .2, .2, .2);
        while (startTimeDrive + .8 >= runtime.seconds() && opModeIsActive()) {
            liftMovement(-.5);
        }
        drive.setMotorPowers(0, 0, 0, 0);
        liftMovement(0);
        dropBlock();
        PIDforward(2);
        liftPower(.7, .5);
        PIDback(56);

    }
    public void runLeft(){
        strafeDistanceSensorFoundation();
        sleep(100);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(26, 20, Math.toRadians(90)))
                        .splineTo(new Pose2d(38, 56, 0))
                        .build()
        );
        grabBlock();
        PIDback(10);
        PIDTurn(-90);
        double startTimeDrive = runtime.seconds();
        drive.setMotorPowers(.5, .5, .5, .5);
        while (startTimeDrive + 2 >= runtime.seconds() && opModeIsActive()) {
            if (runtime.seconds() >= startTimeDrive + 1.4) {
                liftMovement(-.5);
            }
        }
        drive.setMotorPowers(0, 0, 0, 0);
        liftMovement(0);
        PIDstrafeLeft(3);
        dropBlock();
        PIDback(4);
    }
    public void runRight() {
        strafeDistanceSensorFoundation();
        sleep(100);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(26,20,Math.toRadians(90)))
                        .splineTo(new Pose2d(40, 38, 0))
                        .build()
        );
        grabBlock();
        PIDback(10);
        PIDTurn(-90);
        double startTimeDrive = runtime.seconds();
        drive.setMotorPowers(.3, .3, .3, .3);
        while (startTimeDrive + 2 >= runtime.seconds() && opModeIsActive()) {
            if (runtime.seconds() >= startTimeDrive + 1.1) {
                liftMovement(-.3);
            }
        }
        drive.setMotorPowers(0, 0, 0, 0);
        liftMovement(0);
        PIDstrafeLeft(3);
        dropBlock();
        PIDback(4);
    }
    public void strafeDistanceSensorFoundation() {
        if (wallSensorRed.getDistance(DistanceUnit.INCH) <= 26) {
            double startTime = runtime.seconds();
            while (wallSensorRed.getDistance(DistanceUnit.INCH) <= 38 && startTime + .6 >= runtime.seconds() && opModeIsActive()) {
                liftMovement(-.5);
                drive.setMotorPowers(.4, -.4, .4, -.4);
            }

            double startTimedrive = runtime.seconds();
            while (wallSensorRed.getDistance(DistanceUnit.INCH) <= 38 && startTimedrive + .5 >= runtime.seconds() && opModeIsActive()) {
                liftMovement(.65);
                drive.setMotorPowers(.4, -.4, .4, -.4);
            }
            drive.setMotorPowers(0, 0, 0, 0);
            liftMovement(0);
        }
        else {
            double startTimedrive = runtime.seconds();
            while (wallSensorRed.getDistance(DistanceUnit.INCH) <= 38 && startTimedrive + .5 >= runtime.seconds() && opModeIsActive()) {
                liftMovement(-.65);
                drive.setMotorPowers(.4, -.4, .4, -.4);
            }
            drive.setMotorPowers(0, 0, 0, 0);
            liftMovement(0);
            liftPower(.7,.5);

        }
    }
    public void runMiddle() {
        if (opModeIsActive()) {
            strafeDistanceSensorFoundation();
            sleep(100);
            drive.setPoseEstimate(new Pose2d(0, 0, 0));
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(24, 24, Math.toRadians(90)))
                            .splineTo(new Pose2d(34, 53, Math.toRadians(0)))
                            .build()
            );
            sleep(100);
            PIDforward(10);
            sleep(100);
            grabBlock();
            PIDback(7.5);
            sleep(100);
            PIDTurn(-87);
            sleep(100);
            double startTimeDrive = runtime.seconds();
            drive.setMotorPowers(.4, .4, .4, .4);
            while (startTimeDrive + 1.75 >= runtime.seconds() && opModeIsActive()) {
                if (runtime.seconds() >= startTimeDrive + 1) {
                    liftMovement(-.4);
                }
            }
            drive.setMotorPowers(0, 0, 0, 0);
            liftMovement(0);
            PIDstrafeLeft(3);
            dropBlock();
            PIDback(4);

        }
    }
    public void run() {
        if (opModeIsActive()) {
            strafeDistanceSensorFoundation();
            sleep(100);
            drive.setPoseEstimate(new Pose2d(0, 0, 0));
            switch(SS) {
                case 1:
                    //left
                    runLeft();
                    break;
                case 0:
                    //right
                    runRight();
                    break;
                case -1:
                    //middle
                    runMiddle();
                    break;
            }
        }
    }
    public void splineTest() {
        if (opModeIsActive()) {
            double startTime = runtime.seconds();
            while (startTime + .4 >= runtime.seconds() && opModeIsActive()) {
                sleep(250);
                double blockPos = skyStoneDetector.getScreenPosition().x;
                telemetry.addData("Block Pos", blockPos);
                if (blockPos <= 160) {
                    SS = 1;
                    //left
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(34, 26, Math.toRadians(0)))
                                    .build()
                    );
                    sleep(200);
                    PIDforward(1);
                    grabBlock();
                    PIDback(13);
                    PIDTurn(-90);
                    PIDback(8);
                    strafeDistanceSensor();
                    telemetry.addData("block is", "left");
                    telemetry.update();
                } else if (blockPos >= 320) {
                    SS = 0;
                    //right
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(34, 13, Math.toRadians(0)))
                                    .build()
                    );
                    sleep(200);
                    PIDforward(1);
                    grabBlock();
                    PIDback(13);
                    PIDTurn(-90);
                    PIDback(8);
                    strafeDistanceSensor();

                    telemetry.addData("block is", "right");
                    telemetry.update();
                } else {
                    SS = -1;
                    //middle
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(34, 26, Math.toRadians(0)))
                                    .build()
                    );
                    sleep(200);
                    PIDforward(1);
                    grabBlock();
                    PIDback(13);
                    PIDTurn(-90);
                    PIDback(8);
                    strafeDistanceSensor();

                    telemetry.addData("block is", "middle");
                    telemetry.update();

                }
            }
        }
    }
    public void foundation() {
        if (opModeIsActive()) {
            sleep(100);
            double x = drive.getPoseEstimate().getX();
            double y = drive.getPoseEstimate().getY();
            gyroDrive(.8,80,0);
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()

                            .splineTo(new Pose2d(x, -76, Math.toRadians(0)))
                            .build()
            );
            sleep(100);
            double startTimeDrive = runtime.seconds();
            drive.setMotorPowers(.2, .2, .2, .2);
            while (startTimeDrive + .8 >= runtime.seconds() && opModeIsActive()) {
                liftMovement(-.6);
                grabber.setPower(-.5);
            }
            drive.setMotorPowers(0, 0, 0, 0);
            liftMovement(0);
            dropBlock();
            PIDforward(4);
            liftPower(.7, .5);
            double x2 = drive.getPoseEstimate().getX();
            double y2 = drive.getPoseEstimate().getY();
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(x2-5, y2, Math.toRadians(0)))
                            .splineTo(new Pose2d(x2-15, y2, Math.toRadians(0)))
                            .splineTo(new Pose2d(x2-20, y2, Math.toRadians(0)))
                            .splineTo(new Pose2d(x2-25, y2, Math.toRadians(0)))
                            .splineTo(new Pose2d(x2-30, y2, Math.toRadians(0)))
                            .splineTo(new Pose2d(x2-35, y2, Math.toRadians(0)))
                            .splineTo(new Pose2d(x2-40, y2-1, Math.toRadians(0)))
                            .splineTo(new Pose2d(x2-45, y2-1, Math.toRadians(0)))
                            .splineTo(new Pose2d(x2-50, y2-2, Math.toRadians(0)))
                            .splineTo(new Pose2d(x2-55, y2-2, Math.toRadians(0)))
                            .splineTo(new Pose2d(x2-55, y2-3, Math.toRadians(0)))
                            .build()
            );
        }
    }
    public void runtest() {
        if (opModeIsActive()) {
            strafeDistanceSensorFoundation();
            sleep(100);
            drive.setPoseEstimate(new Pose2d(0, 0, 0));
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(24, 24, Math.toRadians(90)))
                            .splineTo(new Pose2d(24, 25, Math.toRadians(90)))
                            .splineTo(new Pose2d(24, 26, Math.toRadians(90)))
                            .splineTo(new Pose2d(24, 27, Math.toRadians(90)))
                            .splineTo(new Pose2d(24, 28, Math.toRadians(90)))
                            .splineTo(new Pose2d(24, 29, Math.toRadians(90)))
                            .splineTo(new Pose2d(34, 57, Math.toRadians(0)))
                            .build()
            );
            sleep(100);
            PIDforward(10);
            sleep(100);
            grabBlock();
            PIDback(16);
            sleep(100);
            PIDTurn(-88);
            sleep(100);
            double startTimeDrive = runtime.seconds();
            drive.setMotorPowers(.4, .4, .4, .4);
            while (startTimeDrive + 1.75 >= runtime.seconds() && opModeIsActive()) {
                if (runtime.seconds() >= startTimeDrive + 1) {
                    liftMovement(-.6);
                }
            }
            drive.setMotorPowers(0, 0, 0, 0);
            liftMovement(0);
            PIDstrafeLeft(3);
            dropBlock();
            PIDback(4);

        }
    }
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            angle += drive.angles.thirdAngle;

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = drive.gyroMotors[1].getCurrentPosition() + moveCounts;
            newRightTarget = drive.gyroMotors[2].getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            drive.gyroMotors[0].setTargetPosition(newLeftTarget);
            drive.gyroMotors[1].setTargetPosition(newLeftTarget);
            drive.gyroMotors[2].setTargetPosition(newRightTarget);
            drive.gyroMotors[3].setTargetPosition(newRightTarget);



            for (int i=0; i<4; i++) {
                drive.gyroMotors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            drive.setGyroPowers(new double[]{speed,speed,speed,speed});


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (drive.gyroMotors[0].isBusy() && drive.gyroMotors[1].isBusy()&&drive.gyroMotors[2].isBusy()&&drive.gyroMotors[3].isBusy())) {

                // adjust relative speed based on heading error.
                error = drive.getError(angle);
                steer = drive.getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                drive.setGyroPowers(new double[]{leftSpeed,leftSpeed,rightSpeed,rightSpeed});

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      drive.gyroMotors[1].getCurrentPosition(),
                        drive.gyroMotors[1].getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            drive.setGyroPowers(new double[]{0,0,0,0});


            // Turn off RUN_TO_POSITION
            for (int i = 0; i < 4; i++) {
                drive.gyroMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        angle += drive.angles.thirdAngle;

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !drive.onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            drive.onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }


        // Stop all motion;
        drive.setGyroPowers(new double[]{0,0,0,0});
    }

}
