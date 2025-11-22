package org.firstinspires.ftc.teamcode.Subsystems.Constants;

public enum MotorConstants {
    SHOOTER_VELOCITY(1300),

    SHOOTER_P(12),
    SHOOTER_I(0),
    SHOOTER_D(0),
    SHOOTER_F(15)
    ;

    public final int value;

    MotorConstants(int value) {
        this.value = value;
    }
}