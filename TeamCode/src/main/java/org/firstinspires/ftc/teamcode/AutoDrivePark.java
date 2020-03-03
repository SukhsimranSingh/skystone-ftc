package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoDrivePark extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    SukhHardware drive;

    MecanumConstraints constraints = new MecanumConstraints(
            new DriveConstraints(30.0, 30.0, 0.0, Math.toRadians(180.0), Math.toRadians(180.0), 0.0),
            18.9, 14.0);


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SukhHardware(hardwareMap);



        // LP = negative moves lift up
        // LP = positive moves lift down

        waitForStart();
        sleep(20000);
        PIDforward(24);




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

   }