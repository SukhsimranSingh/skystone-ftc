package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

public class SukhHardware2 {

    private static final double WHEEL_RADIUS = 2;
    private static double MAX_V              = 60;
    private static double MAX_O              = 60;
    private static double slow_v             = MAX_V / 2;
    private static double slow_o             = MAX_O / 2;
    public DcMotorEx[] motors                = new DcMotorEx[4];
    Orientation lastAngle                    = new Orientation();

    private static Vector2d[] WHEEL_POSITIONS = new Vector2d[]{
            new Vector2d(6.5, 7),       //Left Front
            new Vector2d(-6.5, 7),      //Left Rear
            new Vector2d(-6.5, -7),     //Right Rear
            new Vector2d(6.5, -7)       //Right Front
    };

    private static Vector2d[] ROTOR_DIRECTIONS = {
            new Vector2d(1, -1),
            new Vector2d(1, 1),
            new Vector2d(1, -1),
            new Vector2d(1, 1)
    };

     SukhHardware2(HardwareMap hardwareMap) {

        super();
        motors[0] = hardwareMap.get(DcMotorEx.class, "leftFront");
        motors[1] = hardwareMap.get(DcMotorEx.class, "leftRear");
        motors[2] = hardwareMap.get(DcMotorEx.class, "rightRear");
        motors[3] = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors[0].setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].setDirection(DcMotorEx.Direction.FORWARD);
        motors[2].setDirection(DcMotorEx.Direction.REVERSE);
        motors[3].setDirection(DcMotorEx.Direction.REVERSE);

        for (int i = 0; i < 4; i++)
            motors[i].setPower(0);
    }


//    public void setPower(Pose2d target) {
//        double v = target.vec().norm() * MAX_V;                                                     //Determines the unit vector in the direction of target and then sets the magnitude to MAX_V
//        double theta = Math.atan2(target.getY(), target.getX());                                    //Determines the angle of the unit vector with the horizontal
//        double omega = target.getHeading() * MAX_O;                                                 //Not sure what exactly this does, but my guess is that it receives the heading of the target vector and then sets the magnitude to the max rotation
//        targetVelocity = new Pose2d(v * Math.cos(theta), v * Math.sin(theta), omega);               //Target velocity vector in the form <vx, vy, w>
//        setVelocity(targetVelocity);                                                                //Sets velocity to the target
//    }
//
//    public void setSlowPower(Pose2d target) {                                                       //Same as the setPower function except using low power variables
//        double v = target.vec().norm() * slow_v;
//        double theta = Math.atan2(target.getY(), target.getX());
//        double omega = target.getHeading() * slow_o;
//        targetVelocity = new Pose2d(v * Math.cos(theta), v * Math.sin(theta), omega);
//        setVelocity(targetVelocity);
//    }

    public void setPower (Pose2d target, String power) {                                            //Combines the setPower() and setSlowPower methods by adding an additional argument: "HIGH" or "LOW"
        double v, theta, omega;
        switch (power) {
            case "HIGH":
                v = target.vec().norm() * MAX_V;
                theta = Math.atan2(target.getY(), target.getX());
                omega = target.getHeading() * MAX_O;
                Pose2d targetVelocity = new Pose2d(v * Math.cos(theta), v * Math.sin(theta), omega);
                setVelocity(targetVelocity);
                break;
            case "LOW":
                v = target.vec().norm() * slow_v;
                theta = Math.atan2(target.getY(), target.getX());
                omega = target.getHeading() * slow_o;
                targetVelocity = new Pose2d(v * Math.cos(theta), v * Math.sin(theta), omega);
                setVelocity(targetVelocity);
                break;
        }
    }

    public void setVelocity(Pose2d v) {
        for (int i = 0; i < 4; i++) {
            Vector2d wheelVelocity = new Vector2d(v.getX() - v.getHeading() * WHEEL_POSITIONS[i].getY(),
                                                  v.getY() + v.getHeading() * WHEEL_POSITIONS[i].getX());

            double wheelOmega = (wheelVelocity.dot(ROTOR_DIRECTIONS[i]) * Math.sqrt(2)) / WHEEL_RADIUS;
            motors[i].setVelocity(wheelOmega, AngleUnit.RADIANS);
        }
    }
}

