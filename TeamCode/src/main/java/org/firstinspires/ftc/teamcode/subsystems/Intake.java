package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake{

    DcMotorEx linearSlide;
    DcMotorEx wrist;
    CRServo leftIntake;
    CRServo rightIntake;

    public Intake(HardwareMap hardwareMap){
        this.linearSlide = hardwareMap.get(DcMotorEx.class, "slideMotor");
        this.wrist = hardwareMap.get(DcMotorEx.class, "wrist");
        this.leftIntake = hardwareMap.get(CRServo.class,"intake1");
        this.rightIntake = hardwareMap.get(CRServo.class,"intake2");

        //Configure what the motor does when unpowered (brake or coast)
        wrist.setZeroPowerBehavior(IntakeConstants.WRIST_ZERO_POWER_MODE);
        linearSlide.setZeroPowerBehavior(IntakeConstants.SLIDE_ZERO_POWER_MODE);
        //Configure the motor directions
        wrist.setDirection(IntakeConstants.WRIST_MOTOR_DIRECTION);
        linearSlide.setDirection(IntakeConstants.SLIDE_MOTOR_DIRECTION);
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
        linearSlide.setVelocityPIDFCoefficients(
                IntakeConstants.SLIDE_VELOCITY_P,
                IntakeConstants.SLIDE_I,
                IntakeConstants.SLIDE_D,
                IntakeConstants.SLIDE_F
        );
        //Position PID (uses Velocity coefficients for velocity control when using RUN_WITH_ENCODER)
        wrist.setPositionPIDFCoefficients(IntakeConstants.WRIST_POSITION_P);
        linearSlide.setPositionPIDFCoefficients(IntakeConstants.SLIDE_POSITION_P);
        //Reset Encoder Counts
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void init(){
        //Put initialization code here
    }

    public void update(){
        //Put periodic code here, e.g. telemetry data
    }

    public void setWristPosition(int position){
        wrist.setTargetPosition(position);
        wrist.setVelocity(IntakeConstants.WRIST_SPEED_TICKS_PER_S);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean isWristBusy(){
        return wrist.isBusy();
    }

    public void setLinearSlidePower(double power){
        linearSlide.setPower(power);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean isSlideBusy(){
        return linearSlide.isBusy();
    }

    public void setLinearSlidePosition(int position){
        linearSlide.setTargetPosition(position);
        linearSlide.setVelocity(IntakeConstants.SLIDE_SPEED_TICKS_PER_S);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
    }

    public int getLinearSlidePosition(){
        return linearSlide.getCurrentPosition();
    }

    public double getLinearSlideVelocity(){
        return linearSlide.getVelocity();
    }

}
