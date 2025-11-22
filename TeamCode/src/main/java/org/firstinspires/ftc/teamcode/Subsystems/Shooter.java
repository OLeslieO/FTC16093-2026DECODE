package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Constants.MotorConstants;

public class Shooter {
    public DcMotorEx shooter, preShooter;

    public Shooter(HardwareMap hardwareMap) {
        this.shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        this.preShooter = hardwareMap.get(DcMotorEx.class, "preShooter");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setVelocityPIDFCoefficients(MotorConstants.SHOOTER_P.value, MotorConstants.SHOOTER_I.value, MotorConstants.SHOOTER_D.value, MotorConstants.SHOOTER_F.value);
    }

    public void accelerate(){
        shooter.setVelocity(MotorConstants.SHOOTER_VELOCITY.value);
    }
    public void shoot(){
        preShooter.setPower(0.75);
    }

    public void outtake(){
        preShooter.setPower(-1);
    }

    public void emergency(){
        shooter.setPower(-1);
    }

    public void init(){
        preShooter.setPower(0);
    }

    public void stopAccelerate(){
        shooter.setPower(0);
    }
}