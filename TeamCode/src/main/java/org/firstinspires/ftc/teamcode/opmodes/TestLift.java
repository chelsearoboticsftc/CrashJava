package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
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

            if (gamepad2.b) {
                claw.setLiftPosition(1200);
            }

            telemetry.addData("liftPosition", liftPosition);
            telemetry.addData("liftVelocity", liftVelocity);
            telemetry.addData("liftMaxVelocity", liftMaxVelocity);
            telemetry.update();
        }
    }
}