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
public class RightSideAutonRedTEST extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Pose2d startingPose = new Pose2d(-51,-54,Math.toRadians(45));
        Pose2d parkingPose = new Pose2d(40,-55,Math.toRadians(0));
        Pose2d waypoint = new Pose2d(20,30,0)
        TrajectorySequence deliverSample1 = drivetrain.trajectorySequenceBuilder(startingPose)
                .strafeLeft(12)
                .back(7)
                .build();

        TrajectorySequence parkTrajectory = drivetrain.trajectorySequenceBuilder(deliverSample1.end())
                .splineToLinearHeading(parkingPose,Math.toRadians(0))
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

        TrajectorySequence RightSideAuton = drivetrain.trajectorySequenceBuilder(startingPose)
                .turn(-45)
                .splineTo(waypoint)
                .forward(18)

        //while(opModeIsActive()){
        //Do nothing, wait for end of Op Mode
        //}
    }

}
