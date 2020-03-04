package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;

import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoDriveRed2020 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorSimple liftR;
    private DcMotorSimple liftL;
    private ServoImplEx SSGrabber;
    private CRServo grabber;
    private OpenCvCamera webcamRed;
    private DistanceSensor wallSensorRed;
    private ImprovedSkystoneDetector skyStoneDetector;
    DigitalChannel bottomSwitch;
    double SS;
    SukhHardware drive;

    MecanumConstraints constraints = new MecanumConstraints(
            new DriveConstraints(70.0, 40.0, 0.0, Math.toRadians(180.0), Math.toRadians(180.0), 0.0),
            18.9, 14.0);


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SukhHardware(hardwareMap);


        liftR = hardwareMap.get(DcMotorSimple.class, "liftR");
        liftL = hardwareMap.get(DcMotorSimple.class, "liftL");
        grabber = hardwareMap.get(CRServo.class, "grabber");
        wallSensorRed = hardwareMap.get(DistanceSensor.class,"wallSensorRed");
        bottomSwitch = hardwareMap.get(DigitalChannel.class, "bottomSwitch");
//        SSGrabber =  hardwareMap.get(ServoImplEx.class, "SSGrabber");

        // LP = negative moves lift up
        // LP = positive moves lift down

//        SSGrabber.setPwmRange(new PwmControl.PwmRange(750, 2250));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcamRed = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "webcamRed"), cameraMonitorViewId);

        webcamRed.openCameraDevice();

        skyStoneDetector = new ImprovedSkystoneDetector();
        webcamRed.setPipeline(skyStoneDetector);

        webcamRed.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();



//        splineTest();
//        goToFoundationLine();

        //TODO


        findSkystone();
        foundation();
//        liftPower(-.5, .5);
//        PIDTurn(90);
//        sleep(250);
//        PIDforward(20);
//        dropBlock();
//        liftPower(.6, .25);
//        PIDstrafeLeft(6);
//        PIDback(48);
//        liftPower(-.6, .25);
//        PIDforward(2);
//        PIDback(5);
//        PIDstrafeRight(40);
//        PIDforward(24);
//        PIDTurn(88);
//        getBlock();
//        PIDforward(10);




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
            while (startTime + time >= runtime.seconds() && opModeIsActive()) {
                //do nothing
            }
            liftL.setPower(0);
            liftR.setPower(0);
        }
    }

    public void grabBlock() {
        grabber.setPower(-.75);
        double startTime = runtime.seconds();
        while (startTime + 2 >= runtime.seconds() && opModeIsActive()) {
            //do nothing
        }
        grabber.setPower(0);
    }

    public void goToFoundation() {
        if (opModeIsActive()) {
            if (SS == -1) {
                //middle
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(0, 81))
                                .build());

            } else if (SS == 0) {
                //right
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(0, 73))
                                .build());

            } else if (SS == 1) {
                //left
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(0, 90))
                                .build());
            }
        }
    }

    public void runToFoundation(double val) {
        if (opModeIsActive()) {

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-72, 30, 90))
                            .build()
            );


        }
    }

    public void runToBlock() {
        if (opModeIsActive()) {
            PIDforward(26.5);


        }
    }

    public void releaseBlock() {
        if (opModeIsActive()) {
            dropBlock();
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

    public void runTo2ndSkystone() {
        if (opModeIsActive()) {
            if (SS == -1) {
                //middle
                PIDforward(73);
                PIDTurn(-88);
                getBlock();

            } else if (SS == 0) {
                //right
                PIDforward(66);
                PIDTurn(-88);
                getBlock();

            } else if (SS == 1) {
                //left
                PIDforward(73);
                PIDTurn(-88);
                getBlock();


            }
        }

    }

    public void runTest() {
        if (opModeIsActive()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(72, 36, 10))
                        .build()
        );
    }

    public void runToNewFoundation() {
        if (opModeIsActive()) {

        }


    }




    public void testSpline() {
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-24, 72, Math.toRadians(180)))
                        .build()
        );

    }

    public void moveSkystone() {


        if (opModeIsActive()) {
            if (SS == -1) {
                //middle
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse()
                                .lineTo(new Vector2d(0, 53))
                                .build());
//                dropSkystone();

            } else if (SS == 0) {
                //right
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse()
                                .lineTo(new Vector2d(0, 45))
                                .build());
//                dropSkystone();

            } else if (SS == 1) {
                //left
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse()
                                .lineTo(new Vector2d(0, 61))
                                .build());
//                dropSkystone();
            }
        }

    }

    public void moveFoundationSpline() {
        if (opModeIsActive()) {
//            liftPower(-.25, 2);
//            PIDforward(6);
//            liftPower(.25, 2);
//            if (opModeIsActive()) {
//                grabber.setPower(.75);
//                double startTime = runtime.seconds();
//                while (startTime + 1 >= runtime.seconds() && opModeIsActive()) {
//                    //do nothing
//                }
//                grabber.setPower(0);
//            }
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-24, 24, Math.toRadians(-90)))
                            .build()
            );
        }

    }



    public void getSkystone() {
        if (opModeIsActive()) {
            SSGrabber.setPosition(1);
        }

    }

    public void dropSkystone() {
        if (opModeIsActive()) {
            SSGrabber.setPosition(.5);
        }
    }

    public void runToSkystone2() {
        if (opModeIsActive()) {
            if (SS == -1) {
                //middle
                PIDforward(73);
//                getSkystone();
//                PIDstrafeLeft(4);
//                getSkystone();
//                PIDstrafeRight(10);

            } else if (SS == 0) {
                //right
                PIDforward(66);
//                getSkystone();
//                PIDstrafeLeft(4);
//                getSkystone();
//                PIDstrafeRight(10);

            } else if (SS == 1) {
                //left
                PIDforward(73);
//                getSkystone();
//                PIDstrafeLeft(4);
//                getSkystone();
//                PIDstrafeRight(10);


            }
        }


    }

    public void grabFoundation() {
        grabber.setPower(-.75);
        double startTime = runtime.seconds();
        while (startTime + 3 >= runtime.seconds() && opModeIsActive()) {
            //do nothing
        }
        grabber.setPower(0);
    }

    public void findSkystone() {
        if (opModeIsActive()) {
            double starttime = runtime.seconds();
            while (starttime + .5 >= runtime.seconds() && opModeIsActive()) {
                sleep(250);
                double blockPos = skyStoneDetector.getScreenPosition().x;
                telemetry.addData("Block Pos", blockPos);
                if (blockPos <= 160) {
                    SS = 1;
                    //left
                    PIDstrafeRight(8.5);
                    PIDforward(29);
                    grabBlock();
                    PIDback(8);
                    sleep(100);
                    PIDTurn(-90);
                    sleep(200);
                    strafeDistanceSensor();
                    telemetry.addData("block is", "left");
                    telemetry.update();
                } else if (blockPos >= 320) {
                    SS = 0;
                    //right
                    PIDstrafeLeft(8.5);
                    PIDforward(29);
                    grabBlock();
                    PIDback(8);
                    sleep(100);
                    PIDTurn(-90);
                    sleep(200);
                    strafeDistanceSensor();
                    telemetry.addData("block is", "right");
                    telemetry.update();
                } else {
                    SS = -1;
                    //middle
                    PIDforward(29);
                    grabBlock();
                    PIDback(8);
                    sleep(100);
                    PIDTurn(-90);
                    sleep(200);
                    strafeDistanceSensor();
                    telemetry.addData("block is", "middle");
                    telemetry.update();

                }
            }
        }
    }

    public void splineTest() {
        if (opModeIsActive()) {
            double starttime = runtime.seconds();
            while (starttime + 2 >= runtime.seconds() && opModeIsActive()) {
                sleep(250);
                double blockPos = skyStoneDetector.getScreenPosition().x;
                telemetry.addData("Block Pos", blockPos);
                if (blockPos <= 160) {
                    SS = 1;
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(30, 30, Math.toRadians(86)))
                                    .build()
                    );
                    sleep(250);
                    telemetry.addData("block is", "left");
                    telemetry.update();
                } else if (blockPos >= 320) {
                    SS = 0;
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(30, 19, Math.toRadians(86)))
                                    .build()
                    );
                    sleep(250);
                    telemetry.addData("block is", "right");
                    telemetry.update();
                } else {
                    SS = -1;
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(30, 25, Math.toRadians(86)))
                                    .build()
                    );
                    sleep(250);
                    telemetry.addData("block is", "middle");
                    telemetry.update();

                }
            }
        }
    }

    public void goToFoundationLine() {
        if (opModeIsActive()) {
            if (SS == -1) {
                //middle
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse()
                                .lineTo(new Vector2d(0, 117))
                                .build());

            } else if (SS == 0) {
                //right
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse()
                                .lineTo(new Vector2d(0, 109))
                                .build());

            } else if (SS == 1) {
                //left
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse()
                                .lineTo(new Vector2d(0, 126))
                                .build());
            }
        }
    }

    public void getFoundationSpline() {
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(20, 20, -45))
                        .build()
        );
    }
    public void goToFoundationtest() {
        if (opModeIsActive()) {
            if (SS == -1) {
                //middle
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(0, 57))
                                .build());

            } else if (SS == 0) {
                //right
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(0, 49))
                                .build());

            } else if (SS == 1) {
                //left
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(0, 66))
                                .build());
            }
        }
    }
    public void runToSkystone() {
        if (opModeIsActive()) {
            if (SS == -1) {
                //middle


            } else if (SS == 0) {
                //right

            } else if (SS == 1) {
                //left


            }
        }


    }
    public void findSkystoneSide() {
        if (opModeIsActive()) {
            double starttime = runtime.seconds();
            while (starttime + 2 >= runtime.seconds() && opModeIsActive()) {
                sleep(250);
                double blockPos = skyStoneDetector.getScreenPosition().x;
                telemetry.addData("Block Pos", blockPos);
                if (blockPos <= 160) {
                    SS = 1;
                    PIDstrafeRight(8.5);
                    PIDforward(26);
                    sleep(100);
                    PIDTurn(87);

                    telemetry.addData("block is", "left");
                    telemetry.update();
                } else if (blockPos >= 320) {
                    SS = 0;
                    PIDstrafeLeft(8.5);
                    PIDforward(26);
                    sleep(100);
                    PIDTurn(87);
                    telemetry.addData("block is", "right");
                    telemetry.update();
                } else {
                    SS = -1;
                    PIDforward(26);
                    sleep(100);
                    PIDTurn(87);
                    telemetry.addData("block is", "middle");
                    telemetry.update();

                }
            }
        }
    }
    public void strafeDistanceSensor() {
        while (wallSensorRed.getDistance(DistanceUnit.INCH) < 20.5 && opModeIsActive()) {
            drive.setMotorPowers(.2, -.2, .2, -.2);
        }

    }
    public void foundation() {
        if (opModeIsActive()) {
            sleep(100);
            double x = drive.getPoseEstimate().getX();
            double y = drive.getPoseEstimate().getY();
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(x, -83))
                            .build()
            );
            sleep(200);
            PIDTurn(90);
            sleep(200);
            double startTimeDrive = runtime.seconds();
            drive.setMotorPowers(.2, .2, .2, .2);
            while (startTimeDrive + .8 >= runtime.seconds() && opModeIsActive()) {
                liftMovement(-.4);
                grabber.setPower(-.5);
            }
            drive.setMotorPowers(0, 0, 0, 0);
            liftMovement(0);
            dropBlock();
            PIDforward(4);
            liftPower(.7, .5);
            PIDback(54);
            strafeDistanceSensorFoundation();

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
                            .splineTo(new Pose2d(34, 81, Math.toRadians(0)))
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

}
