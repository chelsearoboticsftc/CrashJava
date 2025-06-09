package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

    //working on:

    //encoders for lift position
    //Bucket servo

    @TeleOp
     public class FieldCentricDrive extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            // Declare our motors
            // Make sure your ID's match your configuration
            Servo bucketServo = hardwareMap.get(Servo.class, "wristservo");

            DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
            DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
            DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
            DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

            DcMotor wrist = hardwareMap.dcMotor.get("wrist");

            DcMotor smokey = hardwareMap.dcMotor.get("smokey");
            smokey.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //this just resets the encoder position at the start of the opmode, so put it in the position you want to be pos. 0.
            smokey.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //declaring servos.
            CRServo intake1 = hardwareMap.crservo.get("intake1");
            CRServo intake2 = hardwareMap.crservo.get("intake2");
            intake2.setDirection(DcMotorSimple.Direction.REVERSE);

            // Reverse the right side motors. This may be wrong for your setup.
            // If your robot moves backwards when commanded to go forwards,
            // reverse the left side instead.
            // See the note about this earlier on this page.
            //frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            // Retrieve the IMU from the hardware map
            IMU imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);

            waitForStart();

            if (isStopRequested()) return;



            //OPMODE
            while (opModeIsActive()) {

                //GAMEPAD1
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.

                if (gamepad1.start) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                //here we plan to add the different mechanisms to the opmode, so that way
                //we can use this as our main opmode for the upcoming FTC season.

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]      This used to be 1 vvv
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), .35);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

                //braking
                frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                //intake rotation
                
                //in
                if(gamepad1.a){
                    wrist.setPower(.8);
                }
                else {
                    wrist.setPower(0);
                }

                //out
                if(gamepad1.b){
                    wrist.setPower(-.8);
                }
                else{
                    wrist.setPower(0);
                }

                //intake suck out
                if(gamepad1.x){
                    intake1.setPower(.7);
                    intake2.setPower(.7);
                }
                else{
                    intake1.setPower(0);
                    intake2.setPower(0);
                }

                //intake suck in
                if(gamepad1.y){
                    intake1.setPower(-.7);
                    intake2.setPower(-.7);
                }
                else {
                    intake1.setPower(0);
                    intake2.setPower(0);
                }

                //GAMEPAD2

                if(gamepad2.right_bumper){
                    smokey.setPower(.75);
                    smokey.getCurrentPosition();
                }else if(gamepad2.left_bumper) {
                    smokey.setPower(-.75);
                    smokey.getCurrentPosition();
                }else{
                    smokey.setPower(-.25);
                    smokey.getCurrentPosition();
                }

                if(gamepad2.x){
                    bucketServo.setPosition(1);
                }else if(gamepad2.y){
                    bucketServo.setPosition(.5);
                }else{
                    //do nothing
                }


            }
        }
    }


