package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.ModernRoboticsMotorControllerParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFPositionParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@MotorType(ticksPerRev=448, gearing=16, maxRPM=337.5, achieveableMaxRPMFraction = .85, orientation= Rotation.CW)
@DeviceProperties(xmlTag="SukhMotor", name="SukhMotor", builtIn = true)
public interface SukhMotor {

}
