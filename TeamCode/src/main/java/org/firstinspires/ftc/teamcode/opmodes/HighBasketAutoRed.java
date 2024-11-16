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
        Pose2d startingPose = new Pose2d(-44.5,-59,Math.toRadians(45));
        Pose2d parkingPose = new Pose2d(40,-55,Math.toRadians(0));
        Pose2d samplegrab1 = new Pose2d(-48,-38, Math.toRadians(90));
        Pose2d deliveryPos = new Pose2d(-57,-57, Math.toRadians(45));
        Pose2d samplegrab2 = new Pose2d(-58, -38,Math.toRadians(90));
        Pose2d samplegrab3 = new Pose2d(-58, -38,Math.toRadians(135));
        TrajectorySequence deliverSample0 = drivetrain.trajectorySequenceBuilder(startingPose)
                .strafeLeft(12)
                .back(7)
                .build();

        /*TrajectorySequence parkTrajectory = drivetrain.trajectorySequenceBuilder(deliverSample1.end())
                .splineToLinearHeading(parkingPose,Math.toRadians(0))
                .build();*/

        TrajectorySequence samplegrab1Traj = drivetrain.trajectorySequenceBuilder(deliverSample0.end())
                .splineToLinearHeading(samplegrab1, Math.toRadians(90))
                .build();

        TrajectorySequence deliverSample1 = drivetrain.trajectorySequenceBuilder(samplegrab1Traj.end())
                .splineToLinearHeading(deliveryPos, Math.toRadians(45))
                .build();

        TrajectorySequence samplegrab2Traj = drivetrain.trajectorySequenceBuilder(deliverSample1.end())
                .splineToLinearHeading(samplegrab2, Math.toRadians(90))
                .build();

        TrajectorySequence deliverSample2 = drivetrain.trajectorySequenceBuilder(samplegrab2Traj.end())
                .splineToLinearHeading(deliveryPos, Math.toRadians(45))
                .build();

        TrajectorySequence samplegrab3Traj = drivetrain.trajectorySequenceBuilder(deliverSample2.end())
                .splineToLinearHeading(samplegrab3, Math.toRadians(135))
                .build();

        TrajectorySequence deliverSample3 = drivetrain.trajectorySequenceBuilder(samplegrab3Traj.end())
                .splineToLinearHeading(deliveryPos, Math.toRadians(45))
                .build();
        drivetrain.setPoseEstimate(startingPose);

        waitForStart();

        //Make sure the claw is closed to start
        claw.setClawPosition(ClawConstants.CLAW_CLOSED);

        drivetrain.followTrajectorySequence(deliverSample0);

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
        //Pick up sample 1
        //TODO:Intake code here
        //Deliver sample 1
        drivetrain.followTrajectorySequence(deliverSample1);
        //Place sample 1
        //TODO:Claw Code here
        drivetrain.followTrajectorySequence(samplegrab2Traj);
        //Pick up sample 2
        //TODO:Intake code here
        //Deliver sample 2
        drivetrain.followTrajectorySequence(deliverSample2);
        //Place sample 2
        //TODO:Claw Code here
        drivetrain.followTrajectorySequence(samplegrab3Traj);
        //Pick up sample 3
        //TODO:Intake code here
        //Deliver sample 3
        drivetrain.followTrajectorySequence(deliverSample3);

        //drivetrain.followTrajectorySequence(parkTrajectory);

        //while(opModeIsActive()){
            //Do nothing, wait for end of Op Mode
        //}
    }

}
