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
    final static public int    WRIST_DELIVER_POS = 100;
    //Servos
    final static DcMotorSimple.Direction LEFT_INTAKE_DIR = DcMotorSimple.Direction.REVERSE;
    final static DcMotorSimple.Direction RIGHT_INTAKE_DIR = DcMotorSimple.Direction.FORWARD;
    public enum IntakeState{
        OFF,
        IN,
        OUT
    }
}
