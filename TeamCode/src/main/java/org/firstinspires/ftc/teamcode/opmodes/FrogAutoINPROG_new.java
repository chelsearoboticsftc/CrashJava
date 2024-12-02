package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class FrogAutoINPROG_new extends LinearOpMode {
    enum State{
        //Preloaded Sample
        DELIVER_SAMPLE_0,
        ROTATE_UP_0,
        ROTATE_DOWN_0,
        //Sample 1
        GRAB_SAMPLE_1,
        DELIVER_SAMPLE_1,
        HAND_OFF_1,
        RAISE_SAMPLE_1,
        ROTATE_UP_1,
        ROTATE_DOWN_1,
        //Sample 2
        GRAB_SAMPLE_2,
        DELIVER_SAMPLE_2,
        HAND_OFF_2,
        RAISE_SAMPLE_2,
        ROTATE_UP_2,
        ROTATE_DOWN_2,
        //Sample 3
        GRAB_SAMPLE_3,
        DELIVER_SAMPLE_3,
        HAND_OFF_3,
        RAISE_SAMPLE_3,
        ROTATE_UP_3,
        ROTATE_DOWN_3,
        //End
        IDLE,
    }
    Pose2d startingPose = new Pose2d(-44.5,-59,Math.toRadians(45));
    Pose2d samplegrab1 = new Pose2d(-48.5,-43, Math.toRadians(90));
    Pose2d deliveryPos = new Pose2d(-56.8,-56.8, Math.toRadians(45));
    Pose2d samplegrab2 = new Pose2d(-62, -45,Math.toRadians(90));
    Pose2d samplegrab3 = new Pose2d(-58, -38,Math.toRadians(135));

    //Define wait times and a timer object
    ElapsedTime waitTimer = new ElapsedTime();
    double rotateUpTime = .5;
    double rotateDownTime = .5;
    double handOffSampleTime = .5;

    //Initialize State Machine
    State currentState = State.IDLE;

    int targetLiftPosition;
    int targetSlidePosition;
    int targetWristPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        TrajectorySequence deliverSample0 = drivetrain.trajectorySequenceBuilder(startingPose)
                .splineToLinearHeading(deliveryPos,Math.toRadians(45))
                .build();

        double turnAngle1 = Math.toRadians(30);
        double turnAngle2 = Math.toRadians(50);
        double turnAngle3 = Math.toRadians(70);

        TrajectorySequence turn1 = drivetrain.trajectorySequenceBuilder(deliverSample0.end())
                .turn(turnAngle1)
                .forward(6)
                .build();

        TrajectorySequence deliverSample1 = drivetrain.trajectorySequenceBuilder(turn1.end())
                .splineToLinearHeading(deliveryPos, Math.toRadians(45))
                .build();


        TrajectorySequence turn2 = drivetrain.trajectorySequenceBuilder(deliverSample1.end())
                .turn(turnAngle2)
                .forward(6)
                .build();

        TrajectorySequence deliverSample2 = drivetrain.trajectorySequenceBuilder(turn2.end())
                .splineToLinearHeading(deliveryPos, Math.toRadians(45))
                .build();

        TrajectorySequence turn3 = drivetrain.trajectorySequenceBuilder(deliverSample2.end())
                .turn(turnAngle3)
                .forward(6)
                .build();

        TrajectorySequence deliverSample3 = drivetrain.trajectorySequenceBuilder(turn3.end())
                .splineToLinearHeading(deliveryPos, Math.toRadians(45))
                .build();

        //Set initial positions
        drivetrain.setPoseEstimate(startingPose);

        waitForStart();

        //Start the first trajectory. Must use asynchronous trajectory following, otherwise it will
        //block the program execution until the trajectory finishes (asynchronous following starts
        //the trajectory following and then returns)
        drivetrain.followTrajectorySequenceAsync(deliverSample0);

        //Set the initial target positions for the lift, linear slide, intake, etc.
        targetLiftPosition = ClawConstants.LIFT_DELIVER_POS;
        targetSlidePosition = IntakeConstants.SLIDE_IN_POS;
        targetWristPosition = IntakeConstants.WRIST_IN_POS;

        //Set state machine to first state

        while((opModeIsActive()) && (!isStopRequested())){
            //Main loop, will run until the end of autonomous, regardless of state machine state.
            switch(currentState){
                //Preloaded Sample
                case DELIVER_SAMPLE_0:
                    if((!drivetrain.isBusy())&&
                       (!claw.isLiftBusy()))
                    {
                        //Rotate the claw up and wait for the move
                        claw.setRotationPosition(ClawConstants.ROTATION_UP);
                        waitTimer.reset();
                        currentState = State.ROTATE_UP_1;
                    }
                    break;
                case ROTATE_UP_0:
                    if(waitTimer.seconds() >= rotateUpTime){
                        //Rotate the claw down and wait for the move
                        claw.setRotationPosition(ClawConstants.ROTATION_DOWN);
                        waitTimer.reset();
                        currentState = State.ROTATE_DOWN_0;
                    }
                    break;
                case ROTATE_DOWN_0:
                    if(waitTimer.seconds() >= rotateDownTime){
                        //Lower the lift, extend the intake, and start turn/move sequence
                        targetLiftPosition = ClawConstants.LIFT_HOME_POS;
                        targetSlidePosition = IntakeConstants.SLIDE_OUT_POS;
                        targetWristPosition = IntakeConstants.WRIST_PICKUP_POS;
                        intake.setIntakeState(IntakeConstants.IntakeState.IN);
                        drivetrain.followTrajectorySequenceAsync(turn1);
                        currentState = State.GRAB_SAMPLE_1;
                    }
                    break;
                //Sample 1
                case GRAB_SAMPLE_1:
                    if((!drivetrain.isBusy())  &&
                       (!claw.isLiftBusy())    &&
                       (!intake.isSlideBusy()) &&
                       (!intake.isWristBusy()))
                    {
                        //Bring the intake in and move to the deliver position (we either have the
                        //sample or we don't!)
                        targetWristPosition = IntakeConstants.WRIST_IN_POS;
                        targetSlidePosition = IntakeConstants.SLIDE_IN_POS;
                        drivetrain.followTrajectorySequenceAsync(deliverSample1);
                        currentState = State.DELIVER_SAMPLE_1;
                    }
                    break;
                case DELIVER_SAMPLE_1:
                    if((!drivetrain.isBusy())  &&
                       (!intake.isSlideBusy()) &&
                       (!intake.isWristBusy()))
                    {
                        //Dump the sample into the claw and then wait
                        intake.setIntakeState(IntakeConstants.IntakeState.OUT);
                        waitTimer.reset();
                        currentState = State.HAND_OFF_1;
                    }
                    break;
                case HAND_OFF_1:
                    if(waitTimer.seconds() >= handOffSampleTime){
                        intake.setIntakeState(IntakeConstants.IntakeState.OFF);
                        targetLiftPosition = ClawConstants.LIFT_DELIVER_POS;
                        currentState = State.RAISE_SAMPLE_1;
                    }
                    break;
                case RAISE_SAMPLE_1:
                    if(!claw.isLiftBusy()){
                        claw.setRotationPosition(ClawConstants.ROTATION_UP);
                        waitTimer.reset();
                        currentState = State.ROTATE_UP_1;
                    }
                    break;
                case ROTATE_UP_1:
                    if(waitTimer.seconds() >= rotateUpTime){
                        //Rotate the claw down and wait for the move
                        claw.setRotationPosition(ClawConstants.ROTATION_DOWN);
                        waitTimer.reset();
                        currentState = State.ROTATE_DOWN_1;
                    }
                    break;
                case ROTATE_DOWN_1:
                    if(waitTimer.seconds() >= rotateDownTime){
                        //Lower the lift, extend the intake, and start turn/move sequence
                        targetLiftPosition = ClawConstants.LIFT_HOME_POS;
                        targetSlidePosition = IntakeConstants.SLIDE_OUT_POS;
                        targetWristPosition = IntakeConstants.WRIST_PICKUP_POS;
                        intake.setIntakeState(IntakeConstants.IntakeState.IN);
                        drivetrain.followTrajectorySequenceAsync(turn2);
                        currentState = State.GRAB_SAMPLE_2;
                    }
                    break;
                //Sample 2
                case GRAB_SAMPLE_2:
                    if((!drivetrain.isBusy())  &&
                            (!claw.isLiftBusy())    &&
                            (!intake.isSlideBusy()) &&
                            (!intake.isWristBusy()))
                    {
                        //Bring the intake in and move to the deliver position (we either have the
                        //sample or we don't!)
                        targetWristPosition = IntakeConstants.WRIST_IN_POS;
                        targetSlidePosition = IntakeConstants.SLIDE_IN_POS;
                        drivetrain.followTrajectorySequenceAsync(deliverSample2);
                        currentState = State.DELIVER_SAMPLE_2;
                    }
                    break;
                case DELIVER_SAMPLE_2:
                    if((!drivetrain.isBusy())  &&
                            (!intake.isSlideBusy()) &&
                            (!intake.isWristBusy()))
                    {
                        //Dump the sample into the claw and then wait
                        intake.setIntakeState(IntakeConstants.IntakeState.OUT);
                        waitTimer.reset();
                        currentState = State.HAND_OFF_2;
                    }
                    break;
                case HAND_OFF_2:
                    if(waitTimer.seconds() >= handOffSampleTime){
                        intake.setIntakeState(IntakeConstants.IntakeState.OFF);
                        targetLiftPosition = ClawConstants.LIFT_DELIVER_POS;
                        currentState = State.RAISE_SAMPLE_2;
                    }
                    break;
                case RAISE_SAMPLE_2:
                    if(!claw.isLiftBusy()){
                        claw.setRotationPosition(ClawConstants.ROTATION_UP);
                        waitTimer.reset();
                        currentState = State.ROTATE_UP_2;
                    }
                    break;
                case ROTATE_UP_2:
                    if(waitTimer.seconds() >= rotateUpTime){
                        //Rotate the claw down and wait for the move
                        claw.setRotationPosition(ClawConstants.ROTATION_DOWN);
                        waitTimer.reset();
                        currentState = State.ROTATE_DOWN_2;
                    }
                    break;
                case ROTATE_DOWN_2:
                    if(waitTimer.seconds() >= rotateDownTime){
                        //Lower the lift, extend the intake, and start turn/move sequence
                        targetLiftPosition = ClawConstants.LIFT_HOME_POS;
                        targetSlidePosition = IntakeConstants.SLIDE_OUT_POS;
                        targetWristPosition = IntakeConstants.WRIST_PICKUP_POS;
                        intake.setIntakeState(IntakeConstants.IntakeState.IN);
                        drivetrain.followTrajectorySequenceAsync(turn3);
                        currentState = State.GRAB_SAMPLE_3;
                    }
                    break;
                //Sample 3
                case GRAB_SAMPLE_3:
                    if((!drivetrain.isBusy())  &&
                            (!claw.isLiftBusy())    &&
                            (!intake.isSlideBusy()) &&
                            (!intake.isWristBusy()))
                    {
                        //Bring the intake in and move to the deliver position (we either have the
                        //sample or we don't!)
                        targetWristPosition = IntakeConstants.WRIST_IN_POS;
                        targetSlidePosition = IntakeConstants.SLIDE_IN_POS;
                        drivetrain.followTrajectorySequenceAsync(deliverSample3);
                        currentState = State.DELIVER_SAMPLE_3;
                    }
                    break;
                case DELIVER_SAMPLE_3:
                    if((!drivetrain.isBusy())  &&
                            (!intake.isSlideBusy()) &&
                            (!intake.isWristBusy()))
                    {
                        //Dump the sample into the claw and then wait
                        intake.setIntakeState(IntakeConstants.IntakeState.OUT);
                        waitTimer.reset();
                        currentState = State.HAND_OFF_3;
                    }
                    break;
                case HAND_OFF_3:
                    if(waitTimer.seconds() >= handOffSampleTime){
                        intake.setIntakeState(IntakeConstants.IntakeState.OFF);
                        targetLiftPosition = ClawConstants.LIFT_DELIVER_POS;
                        currentState = State.RAISE_SAMPLE_3;
                    }
                    break;
                case RAISE_SAMPLE_3:
                    if(!claw.isLiftBusy()){
                        claw.setRotationPosition(ClawConstants.ROTATION_UP);
                        waitTimer.reset();
                        currentState = State.ROTATE_UP_3;
                    }
                    break;
                case ROTATE_UP_3:
                    if(waitTimer.seconds() >= rotateUpTime){
                        //Rotate the claw down and wait for the move
                        claw.setRotationPosition(ClawConstants.ROTATION_DOWN);
                        waitTimer.reset();
                        currentState = State.ROTATE_DOWN_3;
                    }
                    break;
                case ROTATE_DOWN_3:
                    if(waitTimer.seconds() >= rotateDownTime){
                        //Lower the lift, extend the intake, and start turn/move sequence
                        targetLiftPosition = ClawConstants.LIFT_HOME_POS;
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    //Do nothing in Idle
                    break;
            }
        }

        //This part of the code will always run regardless of the switch statement.  Need to update
        //drivetrain, service the PID loops and output telemetry
        drivetrain.update();
        claw.setClawPosition(targetLiftPosition);
        intake.setLinearSlidePosition(targetSlidePosition);
        intake.setWristPosition(targetWristPosition);

        Pose2d poseEstimate = drivetrain.getPoseEstimate();

        telemetry.addData("X Position", poseEstimate.getX());
        telemetry.addData("Y Position", poseEstimate.getY());
        telemetry.addData("Heading", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.addData("Lift Position", claw.getLiftPosition());
        telemetry.addData("Lift Target Position", claw.getLiftTargetPosition());
        telemetry.addData("Slide Position", intake.getLinearSlidePosition());
        telemetry.addData("Slide Target Position", intake.getLinearSlideTargetPosition());
        telemetry.addData("Wrist Position", intake.getWristPosition());
        telemetry.addData("Wrist Target Position", intake.getWristTargetPosition());
        telemetry.addData("State Machine State", currentState);
        telemetry.update();


        /*Old Code
        //Place the preloaded sample
        clawSequence(claw);
        wristOut(intake);
        clawDown(claw);
        //Go to Sample 1 now turn to sample 30 deg
        //drivetrain.turn(Math.toRadians(30));
        //drivetrain.followTrajectorySequence(samplegrab1Traj);
        //Pick up sample 1
        drivetrain.followTrajectorySequence(turn1);
        intakeSequence(intake);
        //Deliver sample 1 turn back
        drivetrain.followTrajectorySequence(deliverSample1);
        //Place sample 1
        clawSequence(claw);
        wristOut(intake);
        clawDown(claw);
        //Go to Sample 2 turn now 20 deg
        //drivetrain.followTrajectorySequence(samplegrab2Traj);
        //Pick up sample 2
        drivetrain.followTrajectorySequence(turn2);
        intakeSequence(intake); //This is where we are currently running out of time (NOT ANYMORE!!!)
        //Deliver sample 2 now turn back 20 deg
        drivetrain.followTrajectorySequence(deliverSample2);
        //Place sample 2
        clawSequence(claw);
        wristOut(intake);
        clawDown(claw);
        //Go to Sample 3 now turn 20 deg
        //drivetrain.followTrajectorySequence(samplegrab3Traj);
        //Pick up sample 3
        drivetrain.followTrajectorySequence(turn3);
        intakeSequence(intake);
        //Deliver sample 3
        drivetrain.followTrajectorySequence(deliverSample3);
        //Place sample 3
        clawSequence(claw);
        wristOut(intake);
        clawDown(claw);

        //while(opModeIsActive()){
        //Do nothing, wait for end of Op Mode
        //}
        */
    }

    public void clawSequence(Claw claw){

        // claw.setClawPosition(ClawConstants.CLAW_CLOSED);

        while(opModeIsActive()){
            claw.setLiftPosition(ClawConstants.LIFT_DELIVER_POS);
            claw.isLiftBusy();
            if(!claw.isLiftBusy()){
                break;
            }
            telemetry.addData("liftPos", claw.getLiftPosition());
            telemetry.update();
        }

        claw.setRotationPosition(ClawConstants.ROTATION_UP);
        //Wait for wrist servo to move to position
        sleep(600);

        //claw.setClawPosition(ClawConstants.CLAW_OPEN);

        sleep( 600);

        //claw.setClawPosition(ClawConstants.CLAW_CLOSED);

        claw.setRotationPosition(ClawConstants.ROTATION_DOWN);
        //Wait for wrist servo to move to position
        sleep(400);

    }

    public void clawDown(Claw claw){
        while(opModeIsActive()){
            claw.setLiftPosition(ClawConstants.LIFT_HOME_POS);
            claw.isLiftBusy();
            if(!claw.isLiftBusy()){
                break;
            }
            //claw.setClawPosition(ClawConstants.CLAW_OPEN);
        }
    }

    public void wristOut(Intake intake){
        while(opModeIsActive()){
            intake.setWristPosition(IntakeConstants.WRIST_PICKUP_POS);
            intake.isWristBusy();
            if(!intake.isWristBusy()){
                break;
            }

        }
    }

    public void intakeSequence(Intake intake){

        while(opModeIsActive()) {
            intake.setLinearSlidePosition(750);
            intake.setIntakeState(IntakeConstants.IntakeState.IN);
            intake.isWristBusy();
            intake.isSlideBusy();
            if((!intake.isWristBusy())&&
               (!intake.isSlideBusy())){
                break;
            }
            telemetry.addData("slidePos", intake.getLinearSlidePosition());
            telemetry.update();
        }

        sleep(300);

        while(opModeIsActive()) {
            intake.setLinearSlidePosition(IntakeConstants.SLIDE_IN_POS);
            intake.isSlideBusy();
            if(!intake.isSlideBusy()){
                break;
            }
            telemetry.addData("slidePos", intake.getLinearSlidePosition());
            telemetry.update();
        }

        sleep(200);

        while(opModeIsActive()) {
            intake.setWristPosition(IntakeConstants.WRIST_IN_POS);
            sleep(1300);
            intake.setIntakeState(IntakeConstants.IntakeState.OUT);
            sleep(800);
            if(!intake.isWristBusy()){
                break;
            }
        }

        intake.setIntakeState(IntakeConstants.IntakeState.OFF);

    }

}
