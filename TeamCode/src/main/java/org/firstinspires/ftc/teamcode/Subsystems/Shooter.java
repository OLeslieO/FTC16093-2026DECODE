package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Constants.MotorConstants;

public class Shooter {
    public DcMotorEx shooterDown, shooterUp, preShooter;

    public Shooter(HardwareMap hardwareMap) {
        this.shooterDown = hardwareMap.get(DcMotorEx.class, "shooterDown");
        this.shooterUp = hardwareMap.get(DcMotorEx.class, "shooterUp");
        this.preShooter = hardwareMap.get(DcMotorEx.class, "preShooter");

        shooterDown.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterUp.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterDown.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterUp.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterUp.setVelocityPIDFCoefficients(MotorConstants.SHOOTER_P.value, MotorConstants.SHOOTER_I.value, MotorConstants.SHOOTER_D.value, MotorConstants.SHOOTER_F.value);
        shooterDown.setVelocityPIDFCoefficients(MotorConstants.SHOOTER_P.value, MotorConstants.SHOOTER_I.value, MotorConstants.SHOOTER_D.value, MotorConstants.SHOOTER_F.value);
    }

    public void accelerate_mid(){
        shooterDown.setVelocity(MotorConstants.SHOOTER_MID_VELOCITY.value);
        shooterUp.setVelocity(MotorConstants.SHOOTER_MID_VELOCITY.value);
    }
    public void accelerate_slow(){
        shooterDown.setVelocity(MotorConstants.SHOOTER_SLOW_VELOCITY.value);
        shooterUp.setVelocity(MotorConstants.SHOOTER_SLOW_VELOCITY.value);
    }
    public void accelerate_fast(){
        shooterDown.setVelocity(MotorConstants.SHOOTER_FAST_VELOCITY.value);
        shooterUp.setVelocity(MotorConstants.SHOOTER_FAST_VELOCITY.value);
    }
    public void shoot(){
        preShooter.setPower(0.6);
    }

    public void outtake(){
        preShooter.setPower(-1);
    }

    public void emergency(){
        shooterDown.setPower(-1);
        shooterUp.setPower(-1);
    }

    public void init(){
        preShooter.setPower(0);
    }

    public void stopAccelerate(){
        shooterDown.setPower(0);
        shooterUp.setPower(0);
    }

    public Runnable autoAccelerate(){
        shooterDown.setVelocity(MotorConstants.SHOOTER_MID_VELOCITY.value);
        shooterUp.setVelocity(MotorConstants.SHOOTER_MID_VELOCITY.value);
        return null;
    }
    public Runnable autoShoot(){
        preShooter.setPower(0.6);
        return null;
    }
    public Runnable autoOuttake(){
        preShooter.setPower(-1);
        return null;
    }
    public Runnable autoEmergency(){
        shooterDown.setPower(-1);
        shooterUp.setPower(-1);
        return null;
    }
    public Runnable autoInit(){
        preShooter.setPower(0);
        return null;
    }
    public Runnable autoStopAccelerate(){
        shooterDown.setPower(0);
        shooterUp.setPower(0);
        return null;
    }
}