package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


@Autonomous
public class RightSideAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        //This needs updating, because I am not sure if the coordinates for the robit are right or not
        Pose2d startingPose = new Pose2d(39.5,62,Math.toRadians(270));
        Vector2d wayPoint1 = new Vector2d(34,24);
        Pose2d parkPosition = new Pose2d(21,11,0);

        TrajectoryVelocityConstraint velocityConstraint;

        drivetrain.setPoseEstimate(startingPose);
        //I will work on recording what we want for our right side auton in more detail, which we will put here. 
        TrajectorySequenceBuilder RightSideAuton = drivetrain.trajectorySequenceBuilder(startingPose);{

        }
    }
}
