package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake{

<<<<<<< Updated upstream
    DcMotorEx linearSlideMotor;
=======
    DcMotorEx slideMotor;
>>>>>>> Stashed changes
    DcMotorEx wrist;
    CRServo leftIntake;
    CRServo rightIntake;

    public Intake(HardwareMap hardwareMap){
<<<<<<< Updated upstream
=======
        this.slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
>>>>>>> Stashed changes
        this.wrist = hardwareMap.get(DcMotorEx.class, "wrist");
        this.leftIntake = hardwareMap.get(CRServo.class,"intake1");
        this.rightIntake = hardwareMap.get(CRServo.class,"intake2");

        //Configure what the motor does when unpowered (brake or coast)
        wrist.setZeroPowerBehavior(IntakeConstants.WRIST_ZERO_POWER_MODE);
<<<<<<< Updated upstream
        //Configure the motor directions
        wrist.setDirection(IntakeConstants.WRIST_MOTOR_DIRECTION);
=======
        slideMotor.setZeroPowerBehavior(IntakeConstants.SLIDE_ZERO_POWER_MODE);
        //Configure the motor directions
        wrist.setDirection(IntakeConstants.WRIST_MOTOR_DIRECTION);
        slideMotor.setDirection(IntakeConstants.SLIDE_MOTOR_DIRECTION);
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
        //Position PID (uses Velocity coefficients for velocity control when using RUN_WITH_ENCODER)
        wrist.setPositionPIDFCoefficients(IntakeConstants.WRIST_POSITION_P);
        //Reset Encoder Counts
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Set motor control type
        //wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
=======
        slideMotor.setVelocityPIDFCoefficients(
                IntakeConstants.SLIDE_VELOCITY_P,
                IntakeConstants.SLIDE_I,
                IntakeConstants.SLIDE_D,
                IntakeConstants.SLIDE_F
        );
        //Position PID (uses Velocity coefficients for velocity control when using RUN_WITH_ENCODER)
        wrist.setPositionPIDFCoefficients(IntakeConstants.WRIST_POSITION_P);
        slideMotor.setPositionPIDFCoefficients(IntakeConstants.SLIDE_POSITION_P);
        //Reset Encoder Counts
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
=======
    public void setLinearSlidePower(double power){
        slideMotor.setPower(power);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLinearSlidePosition(int position){
        slideMotor.setTargetPosition(position);
        slideMotor.setVelocity(IntakeConstants.SLIDE_SPEED_TICKS_PER_S);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
=======
    public int getLinearSlidePosition(){
        return slideMotor.getCurrentPosition();
    }

    public double getLinearSlideVelocity(){
        return slideMotor.getVelocity();
    }

>>>>>>> Stashed changes
}
