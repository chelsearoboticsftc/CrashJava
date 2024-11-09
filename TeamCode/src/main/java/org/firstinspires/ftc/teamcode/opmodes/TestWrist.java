package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeConstants;

@TeleOp
public class TestWrist extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int wristPosition;

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

            telemetry.addData("Wrist Position", wristPosition);
            telemetry.update();
        }
    }
}
