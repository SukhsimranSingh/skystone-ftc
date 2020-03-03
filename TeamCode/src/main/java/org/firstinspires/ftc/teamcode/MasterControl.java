package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MasterControl")
public class MasterControl extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotorSimple liftR = null;
    private DcMotorSimple liftL = null;
    private CRServo grabber = null;
    DigitalChannel bottomSwitch;

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
        grabber = hardwareMap.get(CRServo.class, "grabber");
        bottomSwitch = hardwareMap.get(DigitalChannel.class, "bottomSwitch");

        // set the digital channel to input.
        bottomSwitch.setMode(DigitalChannel.Mode.INPUT);

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
            double sCon = .75;
            double lP = 0;
            double sP = 0;
            double p1 = (yCon * gamepad1.left_stick_y * (-1)) + (xCon * gamepad1.left_stick_x * (-1)) + (rCon * gamepad1.right_stick_x * (+1));
            double p2 = (yCon * gamepad1.left_stick_y * (+1)) + (xCon * gamepad1.left_stick_x * (-1)) + (rCon * gamepad1.right_stick_x * (+1));
            double p3 = (yCon * gamepad1.left_stick_y * (-1)) + (xCon * gamepad1.left_stick_x * (+1)) + (rCon * gamepad1.right_stick_x * (+1));
            double p4 = (yCon * gamepad1.left_stick_y * (+1)) + (xCon * gamepad1.left_stick_x * (+1)) + (rCon * gamepad1.right_stick_x * (+1));
            //             Forward/ Backwards input              Strafe input                           Rotation input

            p1 = Range.clip(p1, -1, 1);
            p2 = Range.clip(p2, -1, 1);
            p3 = Range.clip(p3, -1, 1);
            p4 = Range.clip(p4, -1, 1);
            if (gamepad1.left_trigger > 0) {
                lP = -1 * gamepad1.left_trigger;
            }
            else  if (gamepad1.right_trigger > 0) {
                if (bottomSwitch.getState() == true) {
                    lP = gamepad1.right_trigger;
                    telemetry.addData("limit switch", "Is not Pressed");
                } else if (bottomSwitch.getState() == false) {
                    lP = gamepad1.right_trigger;
                    lP = Range.clip(lP, -1, 0);
                    telemetry.addData("limit switch", "Is pressed");
                }
            }
            else {
                lP = 0;
            }
            if (gamepad1.right_bumper){
                sP = .75;
            }
            else if (gamepad1.left_bumper){
                sP = -.75;
            }
            else {
                sP = 0;
            }


            telemetry.update();

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


