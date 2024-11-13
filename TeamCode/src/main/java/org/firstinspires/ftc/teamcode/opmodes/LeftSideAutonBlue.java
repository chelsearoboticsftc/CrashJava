package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class LeftSideAutonBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        Pose2d startingPose = new Pose2d(39.5, 62, Math.toRadians(270));
        Vector2d wayPoint1 = new Vector2d(34, 24);
        Vector2d waypoint2 = new Vector2d(60, 60);
        Vector2d waypoint3 = new Vector2d(48, 34);
        Vector2d waypoint4 = new Vector2d(54, 34);
        Vector2d parkpoint = new Vector2d(24, 6);
        Pose2d parkPosition = new Pose2d(21, 11, 0);

        TrajectoryVelocityConstraint velocityConstraint;

        drivetrain.setPoseEstimate(startingPose);

        TrajectorySequence smurfNeutralSamples = drivetrain.trajectorySequenceBuilder(startingPose)
                .splineTo(waypoint2, Math.toRadians(45))
                //Deliver to basket
                .splineTo(waypoint3, Math.toRadians(0))
                //Intake
                .splineTo(waypoint2, Math.toRadians(-135))
                //Intake spit
                .splineTo(waypoint4, Math.toRadians(0))
                //Intake in
                .splineTo(waypoint2, Math.toRadians(-135))
                //Spit
                .splineTo(waypoint4, Math.toRadians(-45))
                //In
                .splineTo(waypoint2, Math.toRadians(-135))
                //Spit
                .splineTo(parkpoint, Math.toRadians(90))

                .build();
    }
}