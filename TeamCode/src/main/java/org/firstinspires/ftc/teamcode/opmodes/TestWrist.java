package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeConstants;

@TeleOp
public class   TestWrist extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int wristPosition;
        double maxSlideVelocity = 0;

        Intake intake = new Intake(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            wristPosition = intake.getWristPosition();

            if(gamepad2.y){
                intake.setWristPosition(100);
            }else if(gamepad2.a){
                intake.setWristPosition(0);
            }else{
                //Do nothing!
            }
            if(gamepad2.right_bumper){
                intake.setIntakeState(IntakeConstants.IntakeState.OUT);
            }else{
                intake.setIntakeState(IntakeConstants.IntakeState.OFF);
            }

            if(gamepad2.right_trigger>0.2){
                intake.setLinearSlidePower(0.8);
            }else if(gamepad2.left_trigger>0.2){
                intake.setLinearSlidePower(-0.8);
            }else{
                intake.setLinearSlidePower(0);
            }

            if(gamepad2.b){
                intake.setLinearSlidePosition(IntakeConstants.SLIDE_OUT_POS);
            }else if(gamepad2.x){
                intake.setLinearSlidePosition(IntakeConstants.SLIDE_IN_POS);
            }

            if(Math.abs(intake.getLinearSlideVelocity())>maxSlideVelocity){
                maxSlideVelocity = Math.abs(intake.getLinearSlideVelocity());
            }

            telemetry.addData("Wrist Position", wristPosition);
            telemetry.addData("Slide Position", intake.getLinearSlidePosition());
            telemetry.addData("Max Slide Velocity", maxSlideVelocity);
            telemetry.update();
        }
    }
}
