package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(group = "drive")

public class Auto2020Red2stones extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorSimple liftR;
    private DcMotorSimple liftL;
    private ServoImplEx SSGrabber;
    private CRServo grabber;
    private DistanceSensor wallSensorRed;
    private OpenCvCamera webCamRed;
    private OpenCvCamera webCamBlue;
    private ImprovedSkystoneDetector skyStoneRedDetector;
    private ImprovedSkystoneDetector skyStoneBlueDetector;
    private DigitalChannel bottomSwitch;
    private BNO055IMU imu;
    private double SS;
    private double wallLPos;
    private double wallRPos;
    private SukhHardware drive;

    MecanumConstraints constraints = new MecanumConstraints(
            new DriveConstraints(70.0, 30.0, 0.0, Math.toRadians(180.0), Math.toRadians(180.0), 0.0),
            18.9, 14.0);

    @Override
    public void runOpMode() {

        drive = new SukhHardware(hardwareMap);
        wallSensorRed = hardwareMap.get(DistanceSensor.class, "wallSensorRed");
        liftR = hardwareMap.get(DcMotorSimple.class, "liftR");
        liftL = hardwareMap.get(DcMotorSimple.class, "liftL");
        grabber = hardwareMap.get(CRServo.class, "grabber");
        bottomSwitch = hardwareMap.get(DigitalChannel.class, "bottomSwitch");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
        webCamRed = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webCamRed"), viewportContainerIds[0]);
        skyStoneRedDetector = new ImprovedSkystoneDetector();
        webCamBlue = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webCamBlue"), viewportContainerIds[1]);
        skyStoneBlueDetector = new ImprovedSkystoneDetector();
        webCamRed.openCameraDevice();
        webCamRed.setPipeline(skyStoneRedDetector);
        webCamRed.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        webCamBlue.openCameraDevice();
        webCamBlue.setPipeline(skyStoneBlueDetector);
        webCamBlue.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();
        while(getRuntime() < 30)
            stereoscopicVision();
    }

    private void stereoscopicVision() {
        if (opModeIsActive()) {
            double dTheta = -1 * imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle;
            if (Math.abs(dTheta) > Math.toRadians(5)) {
                PIDTurn(dTheta);
            } else {
                double dY, dX;
                wallLPos = skyStoneRedDetector.getScreenPosition().x;
                wallRPos = skyStoneBlueDetector.getScreenPosition().x;
                double angleA = wallLPos / (320.000 / 52.000) + 64;
                double angleB = wallRPos / (320.000 / 52.000) + 64;
                double angleC = 180.000 - (angleA + angleB);

                angleA = Math.toRadians(angleA);
                angleB = Math.toRadians(angleB);
                angleC = Math.toRadians(angleC);

                double posL = 14.500 * Math.sin(angleB) / Math.sin(angleC);
                double posR = 14.500 * Math.sin(angleA) / Math.sin(angleC);


//                double dY = (posR * posL) / 14.500;
//                double dX = 7.25 - (posL * Math.cos(angleB));
                double diagonal = (Math.pow(posR, 2) + Math.pow(posL, 2)) / 29.0;
                if (angleA > Math.PI / 2.0) {
                    double phi = angleA - (Math.PI / 2.0) + (angleC / 2.0);
                    dY = diagonal * Math.cos(phi);
                    dX = diagonal * Math.sin(phi);
                } else if (angleB > Math.PI / 2.0) {
                    double phi = angleB - (Math.PI / 2.0) + (angleC / 2.0);
                    dY = diagonal * Math.cos(phi);
                    dX = diagonal * Math.sin(phi) * -1;
                } else if (angleA <= Math.PI / 2.0 && angleB <= Math.PI / 2.0 && posR > posL) {
                    double phi = angleB + (angleC / 2.0);
                    dY = diagonal * Math.sin(phi);
                    dX = diagonal * Math.cos(phi);
                } else {
                    double phi = angleA + (angleC / 2.0);
                    dY = diagonal * Math.sin(phi);
                    dX = diagonal * Math.cos(phi) * -1;
                }

                telemetry.addData("posL", posL);
                telemetry.addData("posR", posR);
                telemetry.addData("diagonal", diagonal);
                telemetry.addData("angleA", angleA);
                telemetry.addData("angleB", angleB);
                telemetry.addData("angleC", angleC);
                telemetry.addData("dY", dY);
                telemetry.addData("dX", dX);
                telemetry.addData("dTheta", dTheta);
                telemetry.update();

                double x = drive.getPoseEstimate().getX();
                double y = drive.getPoseEstimate().getY();
                double theta = drive.getPoseEstimate().getHeading();
                sleep(5000);
//            drive.followTrajectorySync(
//                    drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(x + dX, y + dY, theta))
//                            .build()
//            );
            }
        }
    }
    public void PIDTurn(double angle) {

        if (opModeIsActive()) {
            drive.turnSync(angle);
        }
    }
//        splineTest();
//        goToFoundationSpline();
//        findSkystoneNumber();
//        DriveTest();
//        splineTestright();
//        foundation();
//        sleep(100);
//        PIDTurn(90);
//        sleep(100);
//        PIDforward(20);
//        dropBlock();
//        liftPower(.6, .35);
//        PIDstrafeLeft(18);
//        PIDback(48);
//        liftPower(-.6, .25);
//        PIDstrafeRight(34);
//        run();
//        sleep(100);
//        PIDTurn(-87);
//        sleep(100);
//        wall2();
//        sleep(100);
//        PIDTurn(90);
//        PIDforward(24);
//        PIDTurn(86);
//        getBlock();
//        PIDforward(10);
//        findSkystone();
//        foundation();
//        sleep(100);
//        PIDTurn(90);
//        sleep(100);
//        PIDforward(18);
//        dropBlock();
//        liftPower(.6, .28);
//        PIDstrafeLeft(10);
//        PIDback(48);
//        liftPower(-.6, .25);
//        PIDstrafeRight(40);
//        PIDforward(24);
//        PIDTurn(86);
//        getBlock();
//        PIDforward(10);
//    }
//
//
//    public void PIDforward(double Distance) {
//
//        Trajectory trajectory = drive.trajectoryBuilder()
//                .forward(Distance)
//                .build();
//
//        if (opModeIsActive()) {
//
//            if (isStopRequested()) return;
//
//            drive.followTrajectorySync(trajectory);
//        }
//    }
//
//
//    public void PIDwalk() {
//        if (opModeIsActive()) return;
//
//        while (opModeIsActive()) {
//            drive.followTrajectorySync(
//                    drive.trajectoryBuilder()
//                            .forward(48)
//                            .build()
//            );
//        }
//    }
//    public void PIDstrafeLeft(double Distance) {
//
//        Trajectory trajectory = drive.trajectoryBuilder()
//                .strafeLeft(Distance)
//                .build();
//
//        if (opModeIsActive()) {
//            if (isStopRequested()) { return; }
//            drive.followTrajectorySync(trajectory);
//        }
//    }
//
//    public void PIDstrafeRight(double Distance) {
//
//        Trajectory trajectory = drive.trajectoryBuilder()
//                .reverse()
//                .strafeRight(Distance)
//                .build();
//
//        if (opModeIsActive()) {
//
//            if (isStopRequested()) return;
//
//            drive.followTrajectorySync(trajectory);
//        }
//    }
//
//    public void PIDback(double Distance) {
//
//        Trajectory trajectory = drive.trajectoryBuilder()
//                .back(Distance)
//                .build();
//
//        if (opModeIsActive()) {
//
//            if (isStopRequested()) return;
//
//            drive.followTrajectorySync(trajectory);
//        }
//    }
//
//    public void getBlock() {
//        grabber.setPower(-.75);
//        double startTime = runtime.seconds();
//        while (startTime + 1 >= runtime.seconds() && opModeIsActive()) {
//            //do nothing
//        }
//        grabber.setPower(0);
//
//
//    }
//
//    public void liftPower(double lP, double time) {
//        double startTime = runtime.seconds();
//        if (opModeIsActive()) {
//
//            if (bottomSwitch.getState()) {
//                telemetry.addData("limit switch", "Is not Pressed");
//            } else if (!bottomSwitch.getState()) {
//                lP = Range.clip(lP, -1, 0); //motors are reversed
//                telemetry.addData("limit switch", "Is pressed");
//            }
//            liftL.setPower(lP);
//            liftR.setPower(-lP);
//            while (startTime + time >= runtime.seconds()) {
//                //do nothing
//            }
//            liftL.setPower(0);
//            liftR.setPower(0);
//        }
//    }
//
//    public void grabBlock() {
//        grabber.setPower(-.75);
//        double Time = runtime.seconds();
//        while (Time + 2 >= runtime.seconds() && opModeIsActive()) {
//            //do nothing
//        }
//        grabber.setPower(0);
//    }
//
//    public void goToFoundation() {
//        if (opModeIsActive()) {
//            if (SS == -1) {
//                //middle
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .lineTo(new Vector2d(0, 56))
//                                .build());
//                liftMovement(-.25);
//                PIDforward(30);
//                liftMovement(0);
//
//            } else if (SS == 0) {
//                //right
//                double liftTime = runtime.seconds();
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .lineTo(new Vector2d(0, 48))
//                                .build());
//                liftMovement(-.25);
//                PIDforward(30);
//                liftMovement(0);
//
//
//            } else if (SS == 1) {
//                //left
//                double liftTime = runtime.seconds();
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .lineTo(new Vector2d(0, 64))
//                                .build());
//                liftMovement(-.25);
//                PIDforward(30);
//                liftMovement(0);
//
//            }
//        }
//    }
//
//    public void runToFoundation(double val) {
//        if (opModeIsActive()) {
//
//            drive.followTrajectorySync(
//                    drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(-72, 30, 90))
//                            .build()
//            );
//
//
//        }
//    }
//
//    public void runToBlock() {
//        if (opModeIsActive()) {
//            PIDforward(26.5);
//
//
//        }
//    }
//
//    public void releaseBlock() {
//        if (opModeIsActive()) {
//            dropBlock();
//        }
//    }
//
//    public void dropBlock() {
//        if (opModeIsActive()) {
//            grabber.setPower(.75);
//            double startTime = runtime.seconds();
//            while (startTime + 1 >= runtime.seconds() && opModeIsActive()) {
//                //do nothing
//            }
//            grabber.setPower(0);
//        }
//    }
//
//    public void runTo2ndSkystone() {
//        if (opModeIsActive()) {
//            if (SS == -1) {
//                //middle
//                PIDforward(73);
//                PIDTurn(-86);
//                getBlock();
//
//            } else if (SS == 0) {
//                //right
//                PIDforward(66);
//                PIDTurn(-86);
//                getBlock();
//
//            } else if (SS == 1) {
//                //left
//                PIDforward(73);
//                PIDTurn(-86);
//                getBlock();
//
//
//            }
//        }
//
//    }
//
//    public void runTest() {
//        if (opModeIsActive()) return;
//
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .splineTo(new Pose2d(72, 36, 10))
//                        .build()
//        );
//    }
//
//    public void runToNewFoundation() {
//        if (opModeIsActive()) {
//
//        }
//
//
//    }
//
//    public void detectSkystone() {
//        if (opModeIsActive()) {
//            double rightBlockDistance = wallSensorRed.getDistance(DistanceUnit.INCH);
//            sleep(250);
//            PIDforward(8);
//            sleep(250);
//            double middleBlockDistance = wallSensorRed.getDistance(DistanceUnit.INCH);
//            if ((middleBlockDistance + 0.7) < rightBlockDistance) {
//                //middle
//                SS = -1;
//                PIDforward(4);
//                PIDTurn(-87);
//                sleep(250);
//                getBlock();
//                PIDback(6);
//                sleep(250);
//                PIDTurn(-87);
//                sleep(100);
//
//            } else if ((middleBlockDistance - 0.7) > rightBlockDistance) {
//                //right
//                SS = 0;
//                PIDback(3);
//                PIDTurn(-87);
//                sleep(250);
//                getBlock();
//                PIDback(6);
//                sleep(250);
//                PIDTurn(-87);
//                sleep(100);
//
//
//            } else {
//                //left
//                SS = 1;
//                PIDforward(11.5);
//                sleep(250);
//                PIDTurn(-87);
//                sleep(250);
//                getBlock();
//                PIDback(6);
//                sleep(250);
//                PIDTurn(-87);
//                sleep(100);
//
//            }
//            telemetry.addData("RB", rightBlockDistance);
//            telemetry.addData("MB", middleBlockDistance);
//            telemetry.update();
//        }
//    }
//
//    public void testSpline() {
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .splineTo(new Pose2d(-24, 72, Math.toRadians(180)))
//                        .build()
//        );
//
//    }
//
//    public void moveSkystone() {
//
//
//        if (opModeIsActive()) {
//            if (SS == -1) {
//                //middle
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .lineTo(new Vector2d(0, 53))
//                                .build());
////                dropSkystone();
//
//            } else if (SS == 0) {
//                //right
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .lineTo(new Vector2d(0, 45))
//                                .build());
////                dropSkystone();
//
//            } else if (SS == 1) {
//                //left
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .lineTo(new Vector2d(0, 61))
//                                .build());
////                dropSkystone();
//            }
//        }
//
//    }
//
//    public void moveFoundationSpline() {
//        if (opModeIsActive()) {
////            liftPower(-.25, 2);
////            PIDforward(6);
////            liftPower(.25, 2);
////            if (opModeIsActive()) {
////                grabber.setPower(.75);
////                double startTime = runtime.seconds();
////                while (startTime + 1 >= runtime.seconds() && opModeIsActive()) {
////                    //do nothing
////                }
////                grabber.setPower(0);
////            }
//            drive.followTrajectorySync(
//                    drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(-24, 24, Math.toRadians(-90)))
//                            .build()
//            );
//        }
//
//    }
//
//    public void findSkystoneDistance() {
//        if (opModeIsActive()) {
//            double rightBlockDistance = wallSensorRed.getDistance(DistanceUnit.INCH);
//            sleep(250);
//            PIDforward(8);
//            sleep(250);
//            double middleBlockDistance = wallSensorRed.getDistance(DistanceUnit.INCH);
//            if ((middleBlockDistance + 0.7) < rightBlockDistance) {
//                //middle
//                SS = -1;
//                PIDback(6);
//                sleep(100);
//                PIDstrafeLeft(4);
//                sleep(100);
//                getBlock();
//                PIDstrafeRight(10);
//
//
//            } else if ((middleBlockDistance - 0.7) > rightBlockDistance) {
//                //right
//                SS = 0;
//                PIDback(14);
//                sleep(100);
//                PIDstrafeLeft(4);
//                sleep(100);
//                getBlock();
//                PIDstrafeRight(10);
//
//
//            } else {
//                //left
//                SS = 1;
//                PIDforward(3);
//                sleep(100);
//                PIDstrafeLeft(4);
//                sleep(100);
//                getBlock();
//                PIDstrafeRight(10);
//
//
//            }
//
//            telemetry.addData("RB", rightBlockDistance);
//            telemetry.addData("MB", middleBlockDistance);
//            telemetry.update();
//        }
//    }
//
//    public void getSkystone() {
//        if (opModeIsActive()) {
//            SSGrabber.setPosition(1);
//        }
//
//    }
//
//    public void dropSkystone() {
//        if (opModeIsActive()) {
//            SSGrabber.setPosition(.5);
//        }
//    }
//
//    public void runToSkystone2() {
//        if (opModeIsActive()) {
//            if (SS == -1) {
//                //middle
//                PIDback(72);
//                PIDTurn(87);
//                sleep(250);
//                PIDforward(5);
//                grabBlock();
//                PIDback(8);
//                PIDTurn(-87);
//
//            } else if (SS == 0) {
//                //right
//                PIDback(72);
//                PIDTurn(87);
//                sleep(250);
//                PIDforward(5);
//                grabBlock();
//                PIDback(8);
//                PIDTurn(-87);
////                getSkystone();
////                PIDstrafeLeft(4);
////                getSkystone();
////                PIDstrafeRight(10);
//
//            } else if (SS == 1) {
//                //left
//                PIDback(72);
//                PIDTurn(87);
//                sleep(250);
//                PIDforward(5);
//                grabBlock();
//                PIDback(8);
//                PIDTurn(-87);
//
//
//            }
//        }
//
//
//    }
//
//    public void grabFoundation() {
//        grabber.setPower(-.75);
//        double startTime = runtime.seconds();
//        while (startTime + 3 >= runtime.seconds() && opModeIsActive()) {
//            //do nothing
//        }
//        grabber.setPower(0);
//    }
//
//    public void findSkystone() {
//        if (opModeIsActive()) {
//            double wallPos = skyStoneRedDetector.getScreenPosition().x;
//
//            if (wallPos <= 160) {
//                SS = 1;
//                PIDstrafeRight(8.5);
//                PIDforward(30);
//                grabBlock();
//                PIDback(8);
//                sleep(100);
//                PIDTurn(-90);
//                sleep(100);
//
//                telemetry.addData("wall is", "left");
//                telemetry.update();
//            } else if (wallPos >= 320) {
//                SS = 0;
//                PIDstrafeLeft(8.5);
//                PIDforward(30);
//                grabBlock();
//                PIDback(8);
//                sleep(100);
//                PIDTurn(-90);
//                sleep(100);
//                telemetry.addData("wall is", "right");
//                telemetry.update();
//            } else {
//                SS = -1;
//                PIDforward(30);
//                grabBlock();
//                PIDback(8);
//                sleep(100);
//                PIDTurn(-90);
//                sleep(100);
//                telemetry.addData("wall is", "middle");
//                telemetry.update();
//
//            }
//        }
//    }
//
//    public void splineTest() {
//        if (opModeIsActive()) {
//            double starttime = runtime.seconds();
//            while (starttime + 2 >= runtime.seconds() && opModeIsActive()) {
//                sleep(250);
//                double wallPos = skyStoneRedDetector.getScreenPosition().x;
//                telemetry.addData("Block Pos", wallPos);
//                if (wallPos <= 160) {
//                    SS = 1;
//                    drive.followTrajectorySync(
//                            drive.trajectoryBuilder()
//                                    .splineTo(new Pose2d(30, 30, 45))
//                                    .build()
//                    );
//                    sleep(250);
//                    PIDTurn(25);
//                    telemetry.addData("wall is", "left");
//                    telemetry.update();
//                } else if (wallPos >= 440) {
//                    SS = 0;
//                    drive.followTrajectorySync(
//                            drive.trajectoryBuilder()
//                                    .splineTo(new Pose2d(30, 19, 45))
//                                    .build()
//                    );
//                    sleep(250);
//                    PIDTurn(25);
//                    telemetry.addData("wall is", "right");
//                    telemetry.update();
//                } else {
//                    SS = -1;
//                    drive.followTrajectorySync(
//                            drive.trajectoryBuilder()
//                                    .splineTo(new Pose2d(30, 25, 45))
//                                    .build()
//                    );
//                    sleep(250);
//                    PIDTurn(25);
//                    telemetry.addData("wall is", "middle");
//                    telemetry.update();
//
//                }
//            }
//        }
//    }
//
//    public void goToFoundationSpline() {
//        if (opModeIsActive()) {
//            if (SS == -1) {
//                //middle
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .lineTo(new Vector2d(0, 117))
//                                .build());
//
//            } else if (SS == 0) {
//                //right
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .lineTo(new Vector2d(0, 109))
//                                .build());
//
//            } else if (SS == 1) {
//                //left
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .lineTo(new Vector2d(0, 126))
//                                .build());
//            }
//        }
//    }
//
//    public void getFoundationSpline() {
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .reverse()
//                        .splineTo(new Pose2d(20, 20, -45))
//                        .build()
//        );
//    }
//
//    public void goToFoundationtest() {
//        if (opModeIsActive()) {
//            if (SS == -1) {
//                //middle
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .lineTo(new Vector2d(0, 57))
//                                .build());
//
//            } else if (SS == 0) {
//                //right
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .lineTo(new Vector2d(0, 49))
//                                .build());
//
//            } else if (SS == 1) {
//                //left
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .lineTo(new Vector2d(0, 66))
//                                .build());
//            }
//        }
//    }
//
//    public void runtoDeliver() {
//        if (opModeIsActive()) {
//            if (SS == -1) {
//                //middle
//                PIDforward(60);
//                dropBlock();
////                getSkystone();
////                PIDstrafeLeft(4);
////                getSkystone();
////                PIDstrafeRight(10);
//
//            } else if (SS == 0) {
//                //right
//                PIDforward(66);
//                dropBlock();
////                getSkystone();
////                PIDstrafeLeft(4);
////                getSkystone();
////                PIDstrafeRight(10);
//
//            } else if (SS == 1) {
//                //left
//                PIDforward(54);
//                dropBlock();
////                getSkystone();
////                PIDstrafeLeft(4);
////                getSkystone();
////                PIDstrafeRight(10);
//
//
//            }
//        }
//
//
//    }
//
//    public void runtoDeliver2() {
//        if (opModeIsActive()) {
//            if (SS == -1) {
//                //middle
//                PIDforward(78);
//                dropBlock();
////                getSkystone();
////                PIDstrafeLeft(4);
////                getSkystone();
////                PIDstrafeRight(10);
//
//            } else if (SS == 0) {
//                //right
//                PIDforward(84);
//                dropBlock();
////                getSkystone();
////                PIDstrafeLeft(4);
////                getSkystone();
////                PIDstrafeRight(10);
//
//            } else if (SS == 1) {
//                //left
//                PIDforward(73);
//                dropBlock();
////                getSkystone();
////                PIDstrafeLeft(4);
////                getSkystone();
////                PIDstrafeRight(10);
//
//
//            }
//        }
//
//
//    }
//
//    public void DriveTest() {
//        if (opModeIsActive()) {
//
//            drive.setPoseEstimate(new Pose2d(-36, -72, Math.toRadians(87)));
//
//            if (SS == -1) {
//                //middle
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .lineTo(new Vector2d(-36, -30))
//                                .build());
//
//            } else if (SS == 0) {
//                //right
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .strafeTo(new Vector2d(-42, -72))
//                                .lineTo(new Vector2d(-42, -30))
//                                .build());
//
//            } else if (SS == 1) {
//                //left
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .strafeTo(new Vector2d(-28, -72))
//                                .lineTo(new Vector2d(-28, -30))
//                                .build());
//
//            }
//        }
//
//    }
//
//    public void findSkystoneNumber() {
//        if (opModeIsActive()) {
//            double starttime = runtime.seconds();
//            while (starttime + 2 >= runtime.seconds() && opModeIsActive()) {
//                sleep(250);
//                double wallPos = skyStoneRedDetector.getScreenPosition().x;
//                telemetry.addData("Block Pos", wallPos);
//                if (wallPos <= 160) {
//                    SS = 1;
//
//                    telemetry.addData("wall is", "left");
//                    telemetry.update();
//                } else if (wallPos >= 320) {
//                    SS = 0;
//
//                    telemetry.addData("wall is", "right");
//                    telemetry.update();
//                } else {
//                    SS = -1;
//
//                    telemetry.addData("wall is", "middle");
//                    telemetry.update();
//
//                }
//            }
//        }
//    }
//
//    public void liftMovement(double lP) {
//        if (opModeIsActive()) {
//
//            if (bottomSwitch.getState()) {
//                telemetry.addData("limit switch", "Is not Pressed");
//            } else if (!bottomSwitch.getState()) {
//                lP = Range.clip(lP, -1, 0); //motors are reversed
//                telemetry.addData("limit switch", "Is pressed");
//            }
//            liftL.setPower(lP);
//            liftR.setPower(-lP);
//
//        }
//    }
//
//    public void foundation() {
//        if (opModeIsActive()) {
//            double startTimeDrive = runtime.seconds();
//            drive.setMotorPowers(.42, .42, .42, .42);
//            while (startTimeDrive + 2.5 >= runtime.seconds() && opModeIsActive()) {
//                if (runtime.seconds() >= startTimeDrive + 1.7) {
//                    liftMovement(-.4);
//                    grabber.setPower(-.5);
//                }
//            }
//            drive.setMotorPowers(0, 0, 0, 0);
//            liftMovement(0);
//            grabber.setPower(0);
//        }
//    }
//
//    public void foundationLeft() {
//        if (opModeIsActive()) {
//            double startTimeDrive = runtime.seconds();
//            drive.setMotorPowers(.45, .45, .45, .45);
//            while (startTimeDrive + 2 >= runtime.seconds() && opModeIsActive()) {
//                if (runtime.seconds() >= startTimeDrive + 1.5) {
//                    liftMovement(-.5);
//                }
//            }
//            drive.setMotorPowers(0, 0, 0, 0);
//            liftMovement(0);
//            while (opModeIsActive() && drive.getPoseEstimate().getY() >= -60) {
//
//            }
//        }
//    }
//
//    public void splineTestright() {
//        if (opModeIsActive()) {
//            double starttime = runtime.seconds();
//            while (starttime + 2 >= runtime.seconds() && opModeIsActive()) {
//                sleep(250);
//                double wallPos = skyStoneRedDetector.getScreenPosition().x;
//                telemetry.addData("Block Pos", wallPos);
//                if (wallPos <= 160) {
//                    SS = 1;
//                    //left
//                    drive.followTrajectorySync(
//                            drive.trajectoryBuilder()
//                                    .splineTo(new Pose2d(34, 22, Math.toRadians(0)))
//                                    .build()
//                    );
//                    sleep(250);
//                    grabBlock();
//                    PIDback(8);
//                    PIDTurn(-92);
//                    sleep(100);
//                    telemetry.addData("wall is", "left");
//                    telemetry.update();
//                } else if (wallPos >= 320) {
//                    SS = 0;
//                    //right
//                    drive.followTrajectorySync(
//                            drive.trajectoryBuilder()
//                                    .splineTo(new Pose2d(34, 13, Math.toRadians(0)))
//                                    .build()
//                    );
//                    sleep(250);
//                    grabBlock();
//                    PIDback(8);
//                    PIDTurn(-92);
//                    sleep(100);
//                    telemetry.addData("wall is", "right");
//                    telemetry.update();
//                } else {
//                    SS = -1;
//                    //middle
//                    drive.followTrajectorySync(
//                            drive.trajectoryBuilder()
//                                    .splineTo(new Pose2d(34, 22, Math.toRadians(0)))
//                                    .build()
//                    );
//                    sleep(250);
//                    grabBlock();
//                    PIDback(8);
//                    PIDTurn(-92);
//                    sleep(100);
//                    telemetry.addData("wall is", "middle");
//                    telemetry.update();
//
//                }
//            }
//        }
//    }
//
//    public void wall2() {
//        if (opModeIsActive()) {
//            PIDTurn(-90);
//            double startTimeDrive = runtime.seconds();
//            drive.setMotorPowers(.4, .4, .4, .4);
//            while (startTimeDrive + 1 >= runtime.seconds() && opModeIsActive()) {
//                liftMovement(-.5);
//            }
//            drive.setMotorPowers(0, 0, 0, 0);
//
//        }
//    }
//
//    public void wall2nd() {
//        if (opModeIsActive()) {
//            if (SS == 0) {
//                //right
//                double x = drive.getPoseEstimate().getX();
//                double y = drive.getPoseEstimate().getY();
//                double H = drive.getPoseEstimate().getHeading();
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .lineTo(new Vector2d(x - 8, y))
//                                .strafeTo(new Vector2d(x - 8, y - 48))
//                                .build()
//                );
//            }
//            if (SS == 1) {
//                //left
//                double x = drive.getPoseEstimate().getX();
//                double y = drive.getPoseEstimate().getY();
//                double H = drive.getPoseEstimate().getHeading();
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .lineTo(new Vector2d(x - 8, y))
//                                .strafeTo(new Vector2d(x - 8, y - 48))
//                                .build()
//                );
//            }
//            if (SS == -1) {
//                //right
//                double x = drive.getPoseEstimate().getX();
//                double y = drive.getPoseEstimate().getY();
//                double H = drive.getPoseEstimate().getHeading();
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .lineTo(new Vector2d(x - 8, y))
//                                .strafeTo(new Vector2d(x - 8, y - 48))
//                                .build()
//                );
//            }
//        }
//    }
//
//    public void run() {
//        if (opModeIsActive()) {
//            if (SS == 0) {
//                //Right
//                PIDstrafeRight(38);
//                liftPower(.6, .35);
//                double x = drive.getPoseEstimate().getX();
//                double y = drive.getPoseEstimate().getY();
//                double H = drive.getPoseEstimate().getHeading();
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .splineTo(new Pose2d((x + 24), (y + 30), (H + Math.toRadians(90))))
//                                .build()
//                );
//
////                drive.followTrajectorySync(
////                        drive.trajectoryBuilder()
////                                .splineTo(new Pose2d((x + 10), (y + 40), (H + Math.toRadians(-90))))
////                                .build()
////                );
//            }
//            if (SS == 1) {
//                //left
//                {
//                    PIDstrafeRight(38);
//                    liftPower(.6, .35);
//                    sleep(100);
//                    double x = drive.getPoseEstimate().getX();
//                    double y = drive.getPoseEstimate().getY();
//                    double H = drive.getPoseEstimate().getHeading();
//                    drive.followTrajectorySync(
//                            drive.trajectoryBuilder()
//                                    .splineTo(new Pose2d((x + 42), (y + 72), (H + Math.toRadians(0))))
//                                    .build()
//                    );
//                }
//            }
//            if (SS == -1) {
//                //middle
//                PIDstrafeRight(38);
//                liftPower(.6, .35);
//                sleep(100);
//                double x = drive.getPoseEstimate().getX();
//                double y = drive.getPoseEstimate().getY();
//                double H = drive.getPoseEstimate().getHeading();
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .splineTo(new Pose2d((x + 34), (y + 72), (H + Math.toRadians(0))))
//                                .build()
//                );
//            }
//        }
//
//    }
//
//    public void foundationspline() {
//        if (opModeIsActive()) {
//            if (SS == 0) {
//                double x = drive.getPoseEstimate().getX();
//                double y = drive.getPoseEstimate().getY();
//                double H = drive.getPoseEstimate().getHeading();
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .splineTo(new Pose2d(34, -20, H + Math.toRadians(87)))
//                                .build()
//                );
//                sleep(100);
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .lineTo(new Vector2d(34, -103))
//                                .build()
//                );
//                sleep(100);
//                PIDstrafeRight(10);
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .splineTo(new Pose2d(25, -92, Math.toRadians(0)))
//                                .build()
//                );
//                sleep(100);
//                double startTimeDrive = runtime.seconds();
//                drive.setMotorPowers(.25, .25, .25, .25);
//                while (startTimeDrive + 1 >= runtime.seconds() && opModeIsActive()) {
//                    liftMovement(-.4);
//                }
//                drive.setMotorPowers(0, 0, 0, 0);
//                liftMovement(0);
//                dropBlock();
//                liftPower(.7, .5);
//
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .lineTo(new Vector2d(-20, -92))
//                                .build()
//                );
//            }
//            if (SS == 1) {
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .splineTo(new Pose2d(34, -20, Math.toRadians(90)))
//                                .build()
//                );
//                sleep(100);
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .lineTo(new Vector2d(34, -100))
//                                .build()
//                );
//                sleep(100);
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .splineTo(new Pose2d(25, -92, Math.toRadians(0)))
//                                .build()
//                );
//                sleep(100);
//                double startTimeDrive = runtime.seconds();
//                drive.setMotorPowers(.25, .25, .25, .25);
//                while (startTimeDrive + 1 >= runtime.seconds() && opModeIsActive()) {
//                    liftMovement(-.4);
//                }
//                drive.setMotorPowers(0, 0, 0, 0);
//                liftMovement(0);
//                dropBlock();
//                liftPower(.6, .5);
//
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .lineTo(new Vector2d(-20, -92))
//                                .build()
//                );
//            }
//            if (SS == -1) {
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .splineTo(new Pose2d(34, -20, Math.toRadians(90)))
//                                .build()
//                );
//                sleep(100);
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .lineTo(new Vector2d(34, -100))
//                                .build()
//                );
//                sleep(100);
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .splineTo(new Pose2d(25, -92, Math.toRadians(0)))
//                                .build()
//                );
//                sleep(100);
//                double startTimeDrive = runtime.seconds();
//                drive.setMotorPowers(.25, .25, .25, .25);
//                while (startTimeDrive + 1 >= runtime.seconds() && opModeIsActive()) {
//                    liftMovement(-.4);
//                }
//                drive.setMotorPowers(0, 0, 0, 0);
//                liftMovement(0);
//                dropBlock();
//                liftPower(.6, .5);
//
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .reverse()
//                                .lineTo(new Vector2d(-20, -92))
//                                .build()
//                );
//            }
//        }
//
//    }
//
//    public void splineTestFull() {
//        if (opModeIsActive()) {
//            sleep(250);
//            double wallPos = skyStoneRedDetector.getScreenPosition().x;
//            telemetry.addData("Block Pos", wallPos);
//
//            if (wallPos <= 160) {
//                SS = 1;
//                webCamRed.stopStreaming();
//                //left
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .splineTo(new Pose2d(36, 22, Math.toRadians(0)))
//                                .build()
//                );
////                stereoscopicVision();
//                grabBlock();
//                sleep(100);
//                telemetry.addData("wall is", "left");
//                telemetry.update();
//            } else if (wallPos >= 320) {
//                SS = 0;
//                webCamRed.stopStreaming();
//
//                //right
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .splineTo(new Pose2d(34, 13, Math.toRadians(0)))
//                                .build()
//                );
////                stereoscopicVision();
//                grabBlock();
//                sleep(100);
//                telemetry.addData("wall is", "right");
//                telemetry.update();
//            } else {
//                SS = -1;
//                webCamRed.stopStreaming();
//
//                //middle
//                drive.followTrajectorySync(
//                        drive.trajectoryBuilder()
//                                .splineTo(new Pose2d(36, 22, Math.toRadians(0)))
//                                .build()
//                );
////                stereoscopicVision();
//                grabBlock();
//                sleep(100);
//                telemetry.addData("wall is", "middle");
//                telemetry.update();
//            }
//        }
//    }
}