package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ClawConstants {
    //Wrist Motor
    final static public DcMotor.ZeroPowerBehavior LIFT_ZERO_POWER_MODE = DcMotor.ZeroPowerBehavior.BRAKE;
    final static public DcMotorSimple.Direction LIFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final static public double LIFT_VELOCITY_P = 1.463;
    final static public double LIFT_I = 0.1463;
    final static public double LIFT_D = 0.0;
    final static public double LIFT_F = 14.63;
    final static public double LIFT_POSITION_P = 5.0; //Open to change
    final static public double LIFT_SPEED_TICKS_PER_S = 2240;
    final static public int LIFT_DELIVER_POS = 1500;
    final static public int LIFT_HOME_POS = 0;
    final static public int LIFT_MAX_POSITION = 1400;
    final static public int LIFT_MIN_POSITION = 0;
    final static public double CLAW_OPEN = 0.5;
    final static public double CLAW_CLOSED = 0.35;
    final static public double ROTATION_DOWN = 0.96;
    final static public double ROTATION_UP = .25;
}

