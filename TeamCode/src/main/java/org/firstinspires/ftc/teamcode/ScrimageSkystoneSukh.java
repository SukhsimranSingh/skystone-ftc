package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.util.Range;

@TeleOp(name="2020DC")
public class ScrimageSkystoneSukh extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotorSimple liftR = null;
    private DcMotorSimple liftL = null;
    private CRServo grabber = null;
    private DcMotorSimple tapeExtender = null;
    private Servo Capstone;
    double servoPosition = 0.16;
    DigitalChannel bottomSwitch;
    DigitalChannel tape;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        liftR = hardwareMap.get(DcMotorSimple.class, "liftR");
        liftL = hardwareMap.get(DcMotorSimple.class, "liftL");
        tapeExtender = hardwareMap.get(DcMotorSimple.class, "tape");

        grabber = hardwareMap.get(CRServo.class, "grabber");
        bottomSwitch = hardwareMap.get(DigitalChannel.class, "bottomSwitch");
        tape = hardwareMap.get(DigitalChannel.class, "tapeDetector");

        Capstone = hardwareMap.servo.get("Capstone");
        Capstone.setPosition(servoPosition);

        // set the digital channel to input.
        bottomSwitch.setMode(DigitalChannel.Mode.INPUT);
        tape.setMode(DigitalChannel.Mode.INPUT);


//        // Most robots need the motor on one side to be reversed to drive forward
//        // Reverse the motor that runs backwards when connected directly to the battery
//        leftFront.setDirection(DcMotor.Direction.FORWARD);
//        leftRear.setDirection(DcMotor.Direction.FORWARD);
//        rightFront.setDirection(DcMotor.Direction.REVERSE);
//        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double yCon = .7; //.7
            double xCon = .8; //.8
            double rCon = 1; //1
            double lP = gamepad2.right_stick_y;
            double sP = (gamepad2.left_stick_y);
            double p1 = (yCon * gamepad1.left_stick_y * (-1)) + (xCon * gamepad1.left_stick_x * (-1)) + (rCon * gamepad1.right_stick_x * (+1));
            double p2 = (yCon * gamepad1.left_stick_y * (+1)) + (xCon * gamepad1.left_stick_x * (-1)) + (rCon * gamepad1.right_stick_x * (+1));
            double p3 = (yCon * gamepad1.left_stick_y * (-1)) + (xCon * gamepad1.left_stick_x * (+1)) + (rCon * gamepad1.right_stick_x * (+1));
            double p4 = (yCon * gamepad1.left_stick_y * (+1)) + (xCon * gamepad1.left_stick_x * (+1)) + (rCon * gamepad1.right_stick_x * (+1));
            //             Forward/ Backwards input              Strafe input                           Rotation input

            p1 = Range.clip(p1, -1, 1);
            p2 = Range.clip(p2, -1, 1);
            p3 = Range.clip(p3, -1, 1);
            p4 = Range.clip(p4, -1, 1);

            if (bottomSwitch.getState() == true) {
                lP = gamepad2.right_stick_y;
                telemetry.addData("limit switch", "Is not Pressed");
            } else if (bottomSwitch.getState() == false) {
                lP = gamepad2.right_stick_y;
                lP = Range.clip(lP, -1, 0);
                telemetry.addData("limit switch", "Is pressed");
            }
            if (gamepad2.left_bumper) {
                tapeExtender.setPower(.8);
            }
            else if (gamepad2.right_bumper) {
                tapeExtender.setPower(-.8);
            }
            else {
                tapeExtender.setPower(0);

            }


            if (tape.getState()) {
                telemetry.addData("switch", "Is not Pressed");
            } else if (!tape.getState()) {
                for (int tapePose = 0; tapePose < 9;tapePose++){
                    tapePose = tapePose + 1;
                    telemetry.addData("cycles",tapePose);

                }
                telemetry.addData("switch", "Is pressed");
            }

            if (gamepad1.left_trigger > 0) {
            servoPosition = gamepad1.left_trigger;
                servoPosition = Range.clip(servoPosition,.16,.75);
                Capstone.setPosition(servoPosition);
                telemetry.addData("Servo pos:", Capstone.getPosition());
            }
            else {
                servoPosition = 0.18;
                Capstone.setPosition(servoPosition);
                telemetry.addData("Servo pos:", Capstone.getPosition());
            }
            telemetry.addData("sP", sP);


            leftFront.setPower(p1);
            rightFront.setPower(p2);
            leftRear.setPower(p3);
            rightRear.setPower(p4);
            grabber.setPower(-sP);
            liftL.setPower(lP);
            liftR.setPower(-lP);

//            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
//            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//            double rightX = gamepad1.right_stick_x;
//            final double v1 = r * Math.cos(robotAngle) + rightX;
//            final double v2 = r * Math.sin(robotAngle) - rightX;
//            final double v3 = r * Math.sin(robotAngle) + rightX;
//            final double v4 = r * Math.cos(robotAngle) - rightX;
//
//            leftFront.setPower(v1);
//            rightFront.setPower(v2);
//            leftRear.setPower(v3);
//            rightRear.setPower(v4);

            telemetry.addData("Grabber", sP);
            telemetry.addData("grabber", (sP > 0) ? "Forward" : "Reverse");
            telemetry.update();
        }
    }
}


