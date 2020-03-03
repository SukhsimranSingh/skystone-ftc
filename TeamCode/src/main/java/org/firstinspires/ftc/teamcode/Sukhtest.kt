package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.util.Angle
import javax.xml.datatype.DatatypeConstants

   object TragectoryGen {
    private val constraints = MecanumConstraints(
            DriveConstraints(30.0,30.0,0.0,Math.toRadians(180.0),Math.toRadians(180.0),0.0),
            18.9, 14.0)

       fun createTrajectory(): ArrayList<Trajectory> {
           return testSplineTurn()
       }
       fun testSplineTurn(): ArrayList<Trajectory>{
           val list = ArrayList<Trajectory>()
           val myPose = Pose2d(-59.0,-35.0,Math.PI)
           var builderθ = TrajectoryBuilder(myPose, constraints)
           builderθ.strafeTo(Vector2d(-59.0,-38.0))
                   .reverse()
                   .splineTo(Pose2d(0.0,-41.0,Math.PI))
                   .splineTo(Pose2d(55.0, -45.0, Math.PI/2))

           list.add(builderθ.build())

           return list
       }
}
