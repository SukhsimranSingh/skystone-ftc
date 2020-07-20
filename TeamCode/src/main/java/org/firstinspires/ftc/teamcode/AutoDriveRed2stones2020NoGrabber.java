package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.openftc.easyopencv.OpenCvWebcam;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoDriveRed2stones2020NoGrabber extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorSimple liftR;
    private DcMotorSimple liftL;
    private ServoImplEx SSGrabber;
    private DistanceSensor wallSensorRed;
    private OpenCvCamera webcamRed;
    private ImprovedSkystoneDetector skyStoneDetector;
    DigitalChannel bottomSwitch;
    int SS;
    SukhHardware drive;

    MecanumConstraints constraints = new MecanumConstraints(
            new DriveConstraints(70.0, 60.0, 0.0, Math.toRadians(180.0), Math.toRadians(180.0), 0.0),
            18.9, 14.0);


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SukhHardware(hardwareMap);


        wallSensorRed = hardwareMap.get(DistanceSensor.class, "wallSensorRed");
        liftR = hardwareMap.get(DcMotorSimple.class, "liftR");
        liftL = hardwareMap.get(DcMotorSimple.class, "liftL");
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
//
        splineTestFull();
        foundationSpline();
        run();

        /**
         * current autonomous**/
//
//        splineTestFull();
//        foundationMiddle();
//        runMiddle();
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
        double startTime = runtime.seconds();
        while (startTime + 1 >= runtime.seconds() && opModeIsActive()) {
            //do nothing
        }


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
        double Time = runtime.seconds();
        while (Time + 2 >= runtime.seconds() && opModeIsActive()) {
            //do nothing
        }
    }

    public void goToFoundation() {
        if (opModeIsActive()) {
            if (SS == -1) {
                //middle
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(0, 56))
                                .build());
                liftMovement(-.25);
                PIDforward(30);
                liftMovement(0);

            } else if (SS == 0) {
                //right
                double liftTime = runtime.seconds();
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(0, 48))
                                .build());
                liftMovement(-.25);
                PIDforward(30);
                liftMovement(0);


            } else if (SS == 1) {
                //left
                double liftTime = runtime.seconds();
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(0, 64))
                                .build());
                liftMovement(-.25);
                PIDforward(30);
                liftMovement(0);

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
            double startTime = runtime.seconds();
            while (startTime + 1 >= runtime.seconds() && opModeIsActive()) {
                //do nothing
            }
        }
    }

    public void runTo2ndSkystone() {
        if (opModeIsActive()) {
            if (SS == -1) {
                //middle
                PIDforward(73);
                PIDTurn(-86);
                getBlock();

            } else if (SS == 0) {
                //right
                PIDforward(66);
                PIDTurn(-86);
                getBlock();

            } else if (SS == 1) {
                //left
                PIDforward(73);
                PIDTurn(-86);
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
                PIDback(72);
                PIDTurn(87);
                sleep(250);
                PIDforward(5);
                grabBlock();
                PIDback(8);
                PIDTurn(-87);

            } else if (SS == 0) {
                //right
                PIDback(72);
                PIDTurn(87);
                sleep(250);
                PIDforward(5);
                grabBlock();
                PIDback(8);
                PIDTurn(-87);
//                getSkystone();
//                PIDstrafeLeft(4);
//                getSkystone();
//                PIDstrafeRight(10);

            } else if (SS == 1) {
                //left
                PIDback(72);
                PIDTurn(87);
                sleep(250);
                PIDforward(5);
                grabBlock();
                PIDback(8);
                PIDTurn(-87);


            }
        }


    }

    public void grabFoundation() {
        double startTime = runtime.seconds();
        while (startTime + 3 >= runtime.seconds() && opModeIsActive()) {
            //do nothing
        }
    }

    public void findSkystone() {
        if (opModeIsActive()) {
            double blockPos = skyStoneDetector.getScreenPosition().x;

            if (blockPos <= 160) {
                SS = 1;
                PIDstrafeRight(8.5);
                PIDforward(30);
                grabBlock();
                PIDback(8);
                sleep(100);
                PIDTurn(-90);
                sleep(100);

                telemetry.addData("block is", "left");
                telemetry.update();
            } else if (blockPos >= 320) {
                SS = 0;
                PIDstrafeLeft(8.5);
                PIDforward(30);
                grabBlock();
                PIDback(8);
                sleep(100);
                PIDTurn(-90);
                sleep(100);
                telemetry.addData("block is", "right");
                telemetry.update();
            } else {
                SS = -1;
                PIDforward(30);
                grabBlock();
                PIDback(8);
                sleep(100);
                PIDTurn(-90);
                sleep(100);
                telemetry.addData("block is", "middle");
                telemetry.update();

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
                                    .splineTo(new Pose2d(30, 30, 45))
                                    .build()
                    );
                    sleep(250);
                    PIDTurn(25);
                    telemetry.addData("block is", "left");
                    telemetry.update();
                } else if (blockPos >= 440) {
                    SS = 0;
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(30, 19, 45))
                                    .build()
                    );
                    sleep(250);
                    PIDTurn(25);
                    telemetry.addData("block is", "right");
                    telemetry.update();
                } else {
                    SS = -1;
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(30, 25, 45))
                                    .build()
                    );
                    sleep(250);
                    PIDTurn(25);
                    telemetry.addData("block is", "middle");
                    telemetry.update();

                }
            }
        }
    }

    public void goToFoundationSpline() {
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

    public void runtoDeliver() {
        if (opModeIsActive()) {
            if (SS == -1) {
                //middle
                PIDforward(60);
                dropBlock();
//                getSkystone();
//                PIDstrafeLeft(4);
//                getSkystone();
//                PIDstrafeRight(10);

            } else if (SS == 0) {
                //right
                PIDforward(66);
                dropBlock();
//                getSkystone();
//                PIDstrafeLeft(4);
//                getSkystone();
//                PIDstrafeRight(10);

            } else if (SS == 1) {
                //left
                PIDforward(54);
                dropBlock();
//                getSkystone();
//                PIDstrafeLeft(4);
//                getSkystone();
//                PIDstrafeRight(10);


            }
        }


    }

    public void runtoDeliver2() {
        if (opModeIsActive()) {
            if (SS == -1) {
                //middle
                PIDforward(78);
                dropBlock();
//                getSkystone();
//                PIDstrafeLeft(4);
//                getSkystone();
//                PIDstrafeRight(10);

            } else if (SS == 0) {
                //right
                PIDforward(84);
                dropBlock();
//                getSkystone();
//                PIDstrafeLeft(4);
//                getSkystone();
//                PIDstrafeRight(10);

            } else if (SS == 1) {
                //left
                PIDforward(73);
                dropBlock();
//                getSkystone();
//                PIDstrafeLeft(4);
//                getSkystone();
//                PIDstrafeRight(10);


            }
        }


    }

    public void DriveTest() {
        if (opModeIsActive()) {

            drive.setPoseEstimate(new Pose2d(-36, -72, Math.toRadians(87)));

            if (SS == -1) {
                //middle
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(-36, -30))
                                .build());

            } else if (SS == 0) {
                //right
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .strafeTo(new Vector2d(-42, -72))
                                .lineTo(new Vector2d(-42, -30))
                                .build());

            } else if (SS == 1) {
                //left
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .strafeTo(new Vector2d(-28, -72))
                                .lineTo(new Vector2d(-28, -30))
                                .build());

            }
        }

    }

    public void findSkystoneNumber() {
        if (opModeIsActive()) {
            double starttime = runtime.seconds();
            while (starttime + 2 >= runtime.seconds() && opModeIsActive()) {
                sleep(250);
                double blockPos = skyStoneDetector.getScreenPosition().x;
                telemetry.addData("Block Pos", blockPos);
                if (blockPos <= 160) {
                    SS = 1;

                    telemetry.addData("block is", "left");
                    telemetry.update();
                } else if (blockPos >= 320) {
                    SS = 0;

                    telemetry.addData("block is", "right");
                    telemetry.update();
                } else {
                    SS = -1;

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

    public void foundation() {
        if (opModeIsActive()) {
            double startTimeDrive = runtime.seconds();
            drive.setMotorPowers(.42, .42, .42, .42);
            while (startTimeDrive + 2.5 >= runtime.seconds() && opModeIsActive()) {
                if (runtime.seconds() >= startTimeDrive + 1.7) {
                    liftMovement(-.4);
                }
            }
            drive.setMotorPowers(0, 0, 0, 0);
            liftMovement(0);
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
            sleep(100);
        }
    }

    public void splineTestright() {
        if (opModeIsActive()) {
            double starttime = runtime.seconds();
            while (starttime + 2 >= runtime.seconds() && opModeIsActive()) {
                sleep(250);
                double blockPos = skyStoneDetector.getScreenPosition().x;
                telemetry.addData("Block Pos", blockPos);
                if (blockPos <= 160) {
                    SS = 1;
                    //left
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(34, 22, Math.toRadians(0)))
                                    .build()
                    );
                    sleep(250);
                    grabBlock();
                    PIDback(8);
                    PIDTurn(-92);
                    sleep(100);
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
                    sleep(250);
                    grabBlock();
                    PIDback(8);
                    PIDTurn(-92);
                    sleep(100);
                    telemetry.addData("block is", "right");
                    telemetry.update();
                } else {
                    SS = -1;
                    //middle
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .splineTo(new Pose2d(34, 22, Math.toRadians(0)))
                                    .build()
                    );
                    sleep(250);
                    grabBlock();
                    PIDback(8);
                    PIDTurn(-92);
                    sleep(100);
                    telemetry.addData("block is", "middle");
                    telemetry.update();

                }
            }
        }
    }

    public void block2() {
        if (opModeIsActive()) {
            double startTimeDrive = runtime.seconds();
            drive.setMotorPowers(-.4, -.4, -.4, -.4);
            while (startTimeDrive + .9 >= runtime.seconds() && opModeIsActive()) {

            }
            drive.setMotorPowers(0, 0, 0, 0);

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
//            if (SS == 1) {
//                //left
//                foundationLeft();
//            } else if (SS == 0) {
//                //right
//                foundationRight();
//            } else if (SS == -1) {
//                //middle
//                foundationMiddle();
//            }


//            while (startTimeDrive + 2.5 >= runtime.seconds() && opModeIsActive()) {
//                if (runtime.seconds() >= startTimeDrive + 1.7) {
//                    liftMovement(-.4);
//                    grabber.setPower(-.5);
//                }
//            }
//            drive.setMotorPowers(0, 0, 0, 0);
//            liftMovement(0);
//            grabber.setPower(0);
        }
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
                                    .addMarker(() -> {
                                        //do something
                                    })
                                    .build()
                    );
                    PIDforward(2);
                    sleep(100);
                    grabBlock();
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
                    telemetry.addData("block is", "middle");
                    telemetry.update();

                }
            }
        }
    }

    public void strafeTime() {
        double startTime = runtime.seconds();
        drive.setMotorPowers(.3, -.3, .3, -.3);
        while (startTime + .6 >= runtime.seconds() && opModeIsActive()) {
            liftMovement(-.5);
            drive.setMotorPowers(.4, -.4, .4, -.4);
        }
        double startTimedrive = runtime.seconds();

        while (startTimedrive + .5 >= runtime.seconds() && opModeIsActive()) {
            liftMovement(.5);
            drive.setMotorPowers(.4, -.4, .4, -.4);
        }
        drive.setMotorPowers(0, 0, 0, 0);
        liftMovement(0);
    }

    public void strafeDistanceSensorFoundation() {
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
        sleep(100);
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

    public void strafeDistanceSensor() {
        while (wallSensorRed.getDistance(DistanceUnit.INCH) < 21.5 && opModeIsActive()) {
            drive.setMotorPowers(.2, -.2, .2, -.2);
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
        PIDback(57);
        sleep(100);
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
    public void runLeft(){
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
}