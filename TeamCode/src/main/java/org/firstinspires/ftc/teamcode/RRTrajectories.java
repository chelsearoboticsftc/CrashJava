package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RRTrajectories extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory goforward = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                .forward(100)
                .build();
        drivetrain.followTrajectory(goforward);

        Trajectory lineToPosition = drivetrain.trajectoryBuilder(new Pose2d(10,0,0))
                .lineTo(new Vector2d(0,0))
                .build();
            drivetrain.followTrajectory(lineToPosition);

            Trajectory strafeleft = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                    .strafeLeft(50)
                    .build();
            drivetrain.followTrajectory(strafeleft);

            Trajectory strafeToPosition = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                    .strafeTo(new Vector2d(0,50))
                    .build();
            drivetrain.followTrajectory(strafeToPosition);

            Trajectory splineToPosition = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(90)))
                    .splineTo(new Vector2d(50,50),0)
                    .build();
            drivetrain.followTrajectory(splineToPosition);

    }
}


