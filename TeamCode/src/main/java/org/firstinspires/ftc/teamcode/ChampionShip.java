package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.util.JoystickTransform;


@TeleOp(name="Championship2020")
public class ChampionShip extends LinearOpMode {

    private ElapsedTime runtime     = new ElapsedTime();
    private int tapePose            = 0;
    private boolean is1YPressed     = false;
    private boolean slowDrive       = false;
    private double servoPosition    = 0.16;
    private DcMotorSimple liftR = null;
    private DcMotorSimple liftL = null;
    private CRServo grabber = null;
    private DcMotorSimple tapeExtender = null;
    private Servo Capstone;
    DigitalChannel bottomSwitch;
    DigitalChannel tapeDetector;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        SukhHardware2 drive = new SukhHardware2(hardwareMap);
        JoystickTransform transform = new JoystickTransform();

//        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
//        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
//        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
//        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        liftL = hardwareMap.get(DcMotorSimple.class, "liftL");
        tapeExtender = hardwareMap.get(DcMotorSimple.class, "tape");
        liftR = hardwareMap.get(DcMotorSimple.class, "liftR");
        grabber = hardwareMap.get(CRServo.class, "grabber");
        bottomSwitch = hardwareMap.get(DigitalChannel.class, "bottomSwitch");
        tapeDetector = hardwareMap.get(DigitalChannel.class, "tapeDetector");

        Capstone = hardwareMap.servo.get("Capstone");
        Capstone.setPosition(servoPosition);

        bottomSwitch.setMode(DigitalChannel.Mode.INPUT);
        tapeDetector.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {                                                                  //Slow Drive Toggle
            if(gamepad1.y){
                if(!is1YPressed){
                    is1YPressed = true;
                    slowDrive = !slowDrive;
                } else{
                    is1YPressed = false;
                }
            }

            Pose2d v;                                                                               //Set Motor Speeds
            if (slowDrive) {
                v = new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
              drive.setPower(v, "LOW");
//              drive.setSlowPower(v);
            } else {
                v = transform.transform(new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));
              drive.setPower(v, "HIGH");
//              drive.setPower(v);
            }

            double lP = gamepad2.right_stick_y;                                                     //Lift Power
            double sP = (0.75 * gamepad2.left_stick_y);                                             //Grabber Power

            if (bottomSwitch.getState()) {
                telemetry.addData("Limit switch", "is not pressed");
                telemetry.update();
            } else {
                lP = Range.clip(lP, -1, 0);
                telemetry.addData("Limit switch", "is pressed");
                telemetry.update();
            }

            if (gamepad2.left_bumper /*&& tapePose < INSERT_MAX_VALUE_HERE*/) {                     //Tape Measure Control
                tapeExtender.setPower(0.8);
                tapePose++;
                telemetry.addData("Cycles: %s", tapePose);
                telemetry.update();
            } else if (gamepad2.right_bumper && tapePose > 0) {
                tapeExtender.setPower(-0.8);
                tapePose--;
                telemetry.addData("Cycles: %s", tapePose);
                telemetry.update();
            } else {
                tapeExtender.setPower(0);
            }

            if (gamepad1.left_trigger > 0) {                                                        //Capstone Servo Control
                servoPosition = gamepad1.left_trigger;
                servoPosition = Range.clip(servoPosition,.16,.75);
                Capstone.setPosition(servoPosition);
                telemetry.addData("Servo pos: %s", Capstone.getPosition());
                telemetry.update();
            } else {
                servoPosition = 0.16;
                Capstone.setPosition(servoPosition);
                telemetry.addData("Servo pos: %s", Capstone.getPosition());
                telemetry.update();
            }

            grabber.setPower(-sP);
            liftL.setPower(lP);
            liftR.setPower(-lP);

            telemetry.addData("Grabber", sP);
            telemetry.addData("grabber", (sP > 0) ? "Forward" : "Reverse");
            telemetry.update();
        }
    }
}


