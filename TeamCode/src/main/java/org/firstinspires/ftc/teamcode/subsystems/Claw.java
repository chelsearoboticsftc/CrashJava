package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    }

    public void init(){
        //Put initialization code here
    }

    public void update(){
        //Put periodic code here, e.g. telemetry data
    }

    public void setLiftPosition(int position){
        liftMotor.setTargetPosition(position);
        liftMotor.setVelocity(ClawConstants.LIFT_SPEED_TICKS_PER_S);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getLiftPosition(){
        return liftMotor.getCurrentPosition();
    }

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

}
