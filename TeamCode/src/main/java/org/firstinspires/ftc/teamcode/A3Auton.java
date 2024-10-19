package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class A3Auton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory goforward = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(36)
                .build();
        drivetrain.followTrajectory(goforward);

        Trajectory strafeRight = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeRight(50)
                .build();
        drivetrain.followTrajectory(strafeRight);

        Trajectory gobackwards = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .back(30)
                .build();
        drivetrain.followTrajectory(gobackwards);
    }
}