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
import org.openftc.easyopencv.OpenCvWebcam;

import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class AutoDriveBlue2020 extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorSimple liftR;
    private DcMotorSimple liftL;
    private ServoImplEx SSGrabber;
    private CRServo grabber;
    private DistanceSensor blockSensor;
    private OpenCvCamera webcamBlue;
    private ImprovedSkystoneDetector skyStoneDetector;
    DigitalChannel bottomSwitch;
    double SS;
    SukhHardware drive;

    MecanumConstraints constraints = new MecanumConstraints(
            new DriveConstraints(60.0, 30.0, 0.0, Math.toRadians(180.0), Math.toRadians(180.0), 0.0),
            18.9, 14.0);


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SukhHardware(hardwareMap);


        blockSensor = hardwareMap.get(DistanceSensor.class, "blockSensor");
        liftR = hardwareMap.get(DcMotorSimple.class, "liftR");
        liftL = hardwareMap.get(DcMotorSimple.class, "liftL");
        grabber = hardwareMap.get(CRServo.class, "grabber");
        bottomSwitch = hardwareMap.get(DigitalChannel.class, "bottomSwitch");
//        SSGrabber =  hardwareMap.get(ServoImplEx.class, "SSGrabber");

        // LP = negative moves lift up
        // LP = positive moves lift down

//        SSGrabber.setPwmRange(new PwmControl.PwmRange(750, 2250));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamBlue = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "webcamBlue"), cameraMonitorViewId);

        webcamBlue.openCameraDevice();

        skyStoneDetector = new ImprovedSkystoneDetector();
        webcamBlue.setPipeline(skyStoneDetector);

        webcamBlue.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        findSkystone();
        goToFoundation();
        liftPower(-.5, .5);
        PIDTurn(-86);
        sleep(250);
        PIDforward(16);
        dropBlock();
        liftPower(.6, .25);
        PIDstrafeLeft(6);
        PIDback(48);
        liftPower(-.6, .25);
        PIDforward(2);
        PIDback(5);
        PIDstrafeLeft(40);
        PIDforward(24);
        PIDTurn(-86);
        getBlock();
        PIDforward(10);


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
        if (opModeIsActive()) {
            grabBlock();
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
                                .lineTo(new Vector2d(0, 90))
                                .build());

            } else if (SS == 0) {
                //right
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(0, 82))
                                .build());

            } else if (SS == 1) {
                //left
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .lineTo(new Vector2d(0, 93))
                                .build());
            }
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
    public void findSkystone() {
        if (opModeIsActive()) {
            double starttime = runtime.seconds();
            while (starttime + 2 >= runtime.seconds() && opModeIsActive()) {
                sleep(250);
                double blockPos = skyStoneDetector.getScreenPosition().x;
                telemetry.addData("Block Pos", blockPos);
                if (blockPos <= 160) {
                    SS = 1;
                    PIDstrafeLeft(8.5);
                    PIDforward(30);
                    grabBlock();
                    PIDback(8);
                    PIDTurn(86);

                    telemetry.addData("block is", "left");
                    telemetry.update();
                } else if (blockPos >= 320) {
                    SS = 0;
                    PIDstrafeRight(8.5);
                    PIDforward(30);
                    grabBlock();
                    PIDback(8);
                    PIDTurn(86);
                    telemetry.addData("block is", "right");
                    telemetry.update();
                } else {
                    SS = -1;
                    PIDforward(30);
                    grabBlock();
                    PIDback(8);
                    PIDTurn(86);
                    telemetry.addData("block is", "middle");
                    telemetry.update();
                }
            }
        }
    }
}
