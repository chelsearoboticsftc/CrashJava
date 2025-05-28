package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
<<<<<<< HEAD

public class Claw {

    DcMotorEx linearSlideMotor;
    DcMotorEx wrist;
    CRServo leftIntake;
    CRServo rightIntake;

    public Claw(HardwareMap hardwareMap){
        this.wrist = hardwareMap.get(DcMotorEx.class, "wrist");
        this.leftIntake = hardwareMap.get(CRServo.class,"intake1");
        this.rightIntake = hardwareMap.get(CRServo.class,"intake2");

        //Configure what the motor does when unpowered (brake or coast)
        wrist.setZeroPowerBehavior(IntakeConstants.WRIST_ZERO_POWER_MODE);
        //Configure the motor directions
        wrist.setDirection(IntakeConstants.WRIST_MOTOR_DIRECTION);
        leftIntake.setDirection(IntakeConstants.LEFT_INTAKE_DIR);
        rightIntake.setDirection(IntakeConstants.RIGHT_INTAKE_DIR);
        //Configure motor PID Coefficients
        //Velocity PIDF
        wrist.setVelocityPIDFCoefficients(
                IntakeConstants.WRIST_VELOCITY_P,
                IntakeConstants.WRIST_I,
                IntakeConstants.WRIST_D,
                IntakeConstants.WRIST_F
        );
        //Position PID (uses Velocity coefficients for velocity control when using RUN_WITH_ENCODER)
        wrist.setPositionPIDFCoefficients(IntakeConstants.WRIST_POSITION_P);
        //Reset Encoder Counts
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Set motor control type
        //wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    DcMotorEx liftMotor;
    Servo claw;
    Servo rotation;

    public Claw(HardwareMap hardwareMap){
        this.liftMotor = hardwareMap.get(DcMotorEx.class, "smokey");
        this.claw = hardwareMap.get(Servo.class, "claw");
        this.rotation = hardwareMap.get(Servo.class, "wristservo");

        //Configure what the motor does when unpowered (brake or coast)
        liftMotor.setZeroPowerBehavior(ClawConstants.LIFT_ZERO_POWER_MODE);
        //Configure the motor directions
        liftMotor.setDirection(ClawConstants.LIFT_MOTOR_DIRECTION);
        //Configure motor PID Coefficients
        //Velocity PIDF
        liftMotor.setVelocityPIDFCoefficients(
                ClawConstants.LIFT_VELOCITY_P,
                ClawConstants.LIFT_I,
                ClawConstants.LIFT_D,
                ClawConstants.LIFT_F
        );
        //Position PID (uses Velocity coefficients for velocity control when using RUN_WITH_ENCODER)
        liftMotor.setPositionPIDFCoefficients(ClawConstants.LIFT_POSITION_P);
        //Reset Encoder Counts
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
>>>>>>> 2fe8859eec1ea3094455fb56903b6531e7072ab3
    }

    public void init(){
        //Put initialization code here
    }

    public void update(){
        //Put periodic code here, e.g. telemetry data
    }

<<<<<<< HEAD
    public void setWristPosition(int position){
        wrist.setTargetPosition(position);
        wrist.setVelocity(IntakeConstants.WRIST_SPEED_TICKS_PER_S);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setIntakeState(IntakeConstants.IntakeState state){
        switch(state){
            case OFF:
            default:
                leftIntake.setPower(0);
                rightIntake.setPower(0);
                break;
            case IN:
                leftIntake.setPower(1);
                rightIntake.setPower(1);
                break;
            case OUT:
                leftIntake.setPower(-1);
                rightIntake.setPower(-1);
                break;
        }
    }

    public int getWristPosition(){
        return wrist.getCurrentPosition();
=======
    public void setLiftPosition(int position){
        liftMotor.setTargetPositionTolerance(70);
        liftMotor.setTargetPosition(position);
        liftMotor.setVelocity(ClawConstants.LIFT_SPEED_TICKS_PER_S);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getLiftPosition(){
        return liftMotor.getCurrentPosition();
    }

    public int getLiftTargetPosition() {return liftMotor.getTargetPosition();};

    public double getClawPosition(){
        return claw.getPosition();
    }

    public double getRotationPosition(){
        return rotation.getPosition();
    }

    public double getLiftVelocity(){
        return liftMotor.getVelocity();
    }

    public void setLiftMotorPower(double power){
        liftMotor.setPower(power);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setClawPosition(double position){
        claw.setPosition(position);
    }

    public void setRotationPosition(double position){
        rotation.setPosition(position);
    }

    public boolean isLiftBusy(){
        return liftMotor.isBusy();
>>>>>>> 2fe8859eec1ea3094455fb56903b6531e7072ab3
    }

}
