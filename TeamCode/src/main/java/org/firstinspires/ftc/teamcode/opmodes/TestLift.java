package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawConstants;

@TeleOp
public class TestLift extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int liftPosition = 0;
        double liftVelocity = 0;
        double liftMaxVelocity = 0;

        Claw claw = new Claw(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            liftPosition = claw.getLiftPosition();
            liftVelocity = claw.getLiftVelocity();
            /*
            if (gamepad2.y) {
                claw.setLiftMotorPower(.8);
            } else if (gamepad2.a) {
                claw.setLiftMotorPower(-.8);
            } else {
                claw.setLiftMotorPower(0);
            }
            if (Math.abs(liftVelocity) > liftMaxVelocity){
                liftMaxVelocity = Math.abs(liftVelocity);
            } */

            if (gamepad2.y) {
                claw.setLiftPosition(ClawConstants.LIFT_DELIVER_POS);
            }else if(gamepad2.a){
                claw.setLiftPosition(ClawConstants.LIFT_MIN_POSITION);
            }

            if(gamepad2.x){
                claw.setClawPosition(ClawConstants.CLAW_OPEN);
            }else if(gamepad2.b){
                claw.setClawPosition(ClawConstants.CLAW_CLOSED);
            }

            if(gamepad2.right_bumper){
                claw.setRotationPosition(ClawConstants.ROTATION_DOWN);
            }else if(gamepad2.left_bumper){
                claw.setRotationPosition(ClawConstants.ROTATION_UP);
            }

            telemetry.addData("liftPosition", liftPosition);
            telemetry.addData("liftVelocity", liftVelocity);
            telemetry.addData("liftMaxVelocity", liftMaxVelocity);
            telemetry.addData("clawPosition", claw.getClawPosition());
            telemetry.addData("rotationPosition", claw.getRotationPosition());
            telemetry.update();
        }
    }
}