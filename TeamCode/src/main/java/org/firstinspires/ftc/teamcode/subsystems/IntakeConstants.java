package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeConstants {
    //Wrist Motor
    final static public DcMotor.ZeroPowerBehavior WRIST_ZERO_POWER_MODE = DcMotor.ZeroPowerBehavior.BRAKE;
    final static public DcMotorSimple.Direction WRIST_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final static public double WRIST_VELOCITY_P = 5.0;
    final static public double WRIST_I = 0.5;
    final static public double WRIST_D = 0.0;
    final static public double WRIST_F = 50.0;
    final static public double WRIST_POSITION_P = 20.0;
    final static public double WRIST_SPEED_TICKS_PER_S = 250;
    final static public int    WRIST_PICKUP_POS = 150;
    final static public int    WRIST_DELIVER_POS = 100;
    final static public int    WRIST_IN_POS = 0;
    //Slide Motor
    final static public DcMotor.ZeroPowerBehavior SLIDE_ZERO_POWER_MODE = DcMotor.ZeroPowerBehavior.BRAKE;
    final static public DcMotorSimple.Direction SLIDE_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final static public double  SLIDE_VELOCITY_P = 2;//1.29;
    final static public double  SLIDE_I = 0.129;
    final static public double  SLIDE_D = 0.0;
    final static public double  SLIDE_F = 12.9004;
    final static public double  SLIDE_POSITION_P = 20.0;
    final static public double  SLIDE_SPEED_TICKS_PER_S = 2240;
    final static public int     SLIDE_OUT_POS = 610;
    final static public int     SLIDE_IN_POS = 0;
    //Servos
    final static DcMotorSimple.Direction LEFT_INTAKE_DIR = DcMotorSimple.Direction.REVERSE;
    final static DcMotorSimple.Direction RIGHT_INTAKE_DIR = DcMotorSimple.Direction.FORWARD;
    public enum IntakeState{
        OFF,
        IN,
        OUT
    }
}
