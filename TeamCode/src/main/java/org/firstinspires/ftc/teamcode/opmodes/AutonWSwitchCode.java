package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
public class AutonWSwitchCode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        //This needs updating, because I am not sure if the coordinates for the robit are right or not
        Pose2d startingPose = new Pose2d(-16,62,Math.toRadians(270));
        Vector2d wayPoint1 = new Vector2d(-8,36);
        Pose2d parkPosition = new Pose2d(21,11,0);
        

        TrajectoryVelocityConstraint velocityConstraint;

        drivetrain.setPoseEstimate(startingPose);
        //I will work on recording what we want for our right side auton in more detail, which we will put here.


        TrajectorySequence RightSideAutonBlue = drivetrain.trajectorySequenceBuilder(startingPose)
                .splineToConstantHeading(wayPoint1,Math.toRadians(270))
                //Deliver specimen to bar
                .strafeRight(29)
                .forward(15)
                //Neutral sample 1
                .strafeRight(14)
                .back(52)
                .forward(53)
                //Neutral sample 2
                .strafeRight(5)
                .back(53)
                .forward(53)
                //Neutral sample 3
                .strafeRight(5)
                .build();

        waitForStart();

        drivetrain.followTrajectorySequence(RightSideAutonBlue);
    }
}