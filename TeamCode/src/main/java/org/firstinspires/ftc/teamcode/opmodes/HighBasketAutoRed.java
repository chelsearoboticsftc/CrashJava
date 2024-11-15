package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class HighBasketAutoRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Pose2d startingPose = new Pose2d(-51,-54,Math.toRadians(45));
        Pose2d parkingPose = new Pose2d(40,-55,Math.toRadians(0));
        Pose2d samplegrab1 = new Pose2d(-47,-39, 0);
        Pose2d deliveryPos = new Pose2d(-62,-62, 45);
        Pose2d samplegrab2 = new Pose2d(-58, -39, 0);
        TrajectorySequence deliverSample1 = drivetrain.trajectorySequenceBuilder(startingPose)
                .strafeLeft(12)
                .back(7)
                .build();

        TrajectorySequence parkTrajectory = drivetrain.trajectorySequenceBuilder(deliverSample1.end())
                .splineToLinearHeading(parkingPose,Math.toRadians(0))
                .build();

        TrajectorySequence samplegrab1Traj = drivetrain.trajectorySequenceBuilder(samplegrab1)
                        .splineToLinearHeading(samplegrab1, Math.toRadians(0))
                                .build();

        TrajectorySequence samplegrab2Traj = drivetrain.trajectorySequenceBuilder(samplegrab2)
                .splineToLinearHeading(samplegrab2, Math.toRadians(0))
                .build();

        TrajectorySequence deliveryPosMove = drivetrain.trajectorySequenceBuilder(deliveryPos)
                .splineToLinearHeading(samplegrab2, Math.toRadians(0))
                .build();
        drivetrain.setPoseEstimate(startingPose);

        waitForStart();

        //Make sure the claw is closed to start
        claw.setClawPosition(ClawConstants.CLAW_CLOSED);

        drivetrain.followTrajectorySequence(deliverSample1);

        while(opModeIsActive()){
            claw.setLiftPosition(ClawConstants.LIFT_DELIVER_POS);
            claw.isLiftBusy();
            if(!claw.isLiftBusy()){
                break;
            }
        }

        claw.setRotationPosition(ClawConstants.ROTATION_UP);
        //Wait for wrist servo to move to position
        sleep(1500);

        claw.setClawPosition(ClawConstants.CLAW_OPEN);

        sleep( 1000);

        claw.setClawPosition(ClawConstants.CLAW_CLOSED);

        claw.setRotationPosition(ClawConstants.ROTATION_DOWN);
        //Wait for wrist servo to move to position
        sleep(1500);

        while(opModeIsActive()){
            claw.setLiftPosition(ClawConstants.LIFT_HOME_POS);
            claw.isLiftBusy();
            if(!claw.isLiftBusy()){
                break;
            }
            telemetry.addData("position",claw.getLiftPosition());
            telemetry.update();
        }

        drivetrain.followTrajectorySequence(samplegrab1Traj);
        //
        drivetrain.followTrajectorySequence(deliveryPosMove);
        drivetrain.followTrajectorySequence(samplegrab2Traj);
        drivetrain.followTrajectorySequence(deliveryPosMove);
        drivetrain.followTrajectorySequence(parkTrajectory);

        //while(opModeIsActive()){
            //Do nothing, wait for end of Op Mode
        //}
    }

}
