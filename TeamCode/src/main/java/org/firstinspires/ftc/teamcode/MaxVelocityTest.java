package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
    public class MaxVelocityTest extends LinearOpMode {
        DcMotorEx leftFront;
        DcMotorEx leftRear;
        DcMotorEx rightRear;
        DcMotorEx rightFront;


    double currentVelocity;
        double maxVelocity = 0.0;

        @Override
        public void runOpMode() {
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
            rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");


            waitForStart();

            while (opModeIsActive()) {
                currentVelocity = leftFront.getVelocity();

                leftRear.setPower(1);
                leftFront.setPower(1);
                rightRear.setPower(1);
                rightFront.setPower(-1);

                if (currentVelocity > maxVelocity) {
                    maxVelocity = currentVelocity;
                }

                telemetry.addData("current velocity", currentVelocity);
                telemetry.addData("maximum velocity", maxVelocity);
                telemetry.update();
            }

            //stop all motors
            leftRear.setPower(0);
            leftFront.setPower(0);
            rightRear.setPower(0);
            rightFront.setPower(0);
            sleep(10000);
        }
    }