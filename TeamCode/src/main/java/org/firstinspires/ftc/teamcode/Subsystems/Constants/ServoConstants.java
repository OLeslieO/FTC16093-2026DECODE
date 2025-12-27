package org.firstinspires.ftc.teamcode.Subsystems.Constants;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

public enum ServoConstants {

    SHOOTER_TURRET_SLOW(0.67),
    SHOOTER_TURRET_MID(0.72),
    SHOOTER_TURRET_LONG(0.78),

    ;



    public final double value;

    ServoConstants(double pos) {
        value = pos;
    }
    public void setToServo(@NonNull Servo servo){
        servo.setPosition(this.value);
    }
}