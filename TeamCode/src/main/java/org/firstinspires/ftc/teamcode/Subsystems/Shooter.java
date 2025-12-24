package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.Subsystems.Constants.MotorConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Constants.ServoConstants;
import org.firstinspires.ftc.teamcode.testing.ShooterTest;

public class Shooter {
    public DcMotorEx shooterDown, shooterUp, preShooter;
    public  Servo shooterRight, shooterLeft;

    public double targetVelocity;
    public double currentVelocity;

    public Shooter(HardwareMap hardwareMap) {
        this.shooterDown = hardwareMap.get(DcMotorEx.class, "shooterDown");
        this.shooterUp = hardwareMap.get(DcMotorEx.class, "shooterUp");
        this.preShooter = hardwareMap.get(DcMotorEx.class, "preShooter");
        this.shooterRight = hardwareMap.get(Servo.class,"shooterRight");
        this.shooterLeft = hardwareMap.get(Servo.class,"shooterLeft");
//        this.indicatorLight = hardwareMap.get(Servo.class,"indicatorLight");

        shooterDown.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterUp.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(Servo.Direction.FORWARD);
        shooterLeft.setDirection(Servo.Direction.REVERSE);


        shooterDown.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterUp.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        preShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shooterDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterUp.setVelocityPIDFCoefficients(MotorConstants.SHOOTER_P.value, MotorConstants.SHOOTER_I.value, MotorConstants.SHOOTER_D.value, MotorConstants.SHOOTER_F.value);
        shooterDown.setVelocityPIDFCoefficients(MotorConstants.SHOOTER_P.value, MotorConstants.SHOOTER_I.value, MotorConstants.SHOOTER_D.value, MotorConstants.SHOOTER_F.value);
    }

    public void autoLongshoot() {
        currentVelocity = shooterDown.getVelocity();
        shooterDown.setVelocity(MotorConstants.SHOOTER_FAST_VELOCITY.value);
        shooterUp.setVelocity(MotorConstants.SHOOTER_FAST_VELOCITY.value);
        targetVelocity = MotorConstants.SHOOTER_FAST_VELOCITY.value;
        if (Math.abs(currentVelocity - targetVelocity) <= 40) {
            preShooter.setPower(1);
        } else {
            preShooter.setPower(0);
        }
    }
    public void autoMidshoot() {
        currentVelocity = shooterDown.getVelocity();
        shooterDown.setVelocity(MotorConstants.SHOOTER_MID_VELOCITY.value);
        shooterUp.setVelocity(MotorConstants.SHOOTER_MID_VELOCITY.value);
        targetVelocity = MotorConstants.SHOOTER_MID_VELOCITY.value;
        if (Math.abs(currentVelocity - targetVelocity) <= 40) {
            preShooter.setPower(1);
        } else {
            preShooter.setPower(0);
        }
    }

    public void autoShortshoot() {
        currentVelocity = shooterDown.getVelocity();
        shooterDown.setVelocity(MotorConstants.SHOOTER_SLOW_VELOCITY.value);
        shooterUp.setVelocity(MotorConstants.SHOOTER_SLOW_VELOCITY.value);
        targetVelocity = MotorConstants.SHOOTER_SLOW_VELOCITY.value;
        if (Math.abs(currentVelocity - targetVelocity) <= 40) {
            preShooter.setPower(1);
        } else {
            preShooter.setPower(0);
        }
    }
    public void auto_accelerate_slow(){
        shooterDown.setVelocity(MotorConstants.SHOOTER_AUTO_SLOW_VELOCITY.value);
        shooterUp.setVelocity(MotorConstants.SHOOTER_AUTO_SLOW_VELOCITY.value);
    }
    public void accelerate_mid(){

        shooterDown.setVelocity(MotorConstants.SHOOTER_MID_VELOCITY.value);
        shooterUp.setVelocity(MotorConstants.SHOOTER_MID_VELOCITY.value);
        shooterRight.setPosition(ServoConstants.SHOOTER_TURRET_MID.value);
        shooterLeft.setPosition(ServoConstants.SHOOTER_TURRET_MID.value - 0.1);


    }
    public void accelerate_slow(){
        shooterDown.setVelocity(MotorConstants.SHOOTER_SLOW_VELOCITY.value);
        shooterUp.setVelocity(MotorConstants.SHOOTER_SLOW_VELOCITY.value);
        shooterRight.setPosition(ServoConstants.SHOOTER_TURRET_SLOW.value);
        shooterLeft.setPosition(ServoConstants.SHOOTER_TURRET_SLOW.value - 0.1);
    }
    public void accelerate_fast(){
        shooterDown.setVelocity(MotorConstants.SHOOTER_FAST_VELOCITY.value);
        shooterUp.setVelocity(MotorConstants.SHOOTER_FAST_VELOCITY.value);
        shooterRight.setPosition(ServoConstants.SHOOTER_TURRET_LONG.value);
        shooterLeft.setPosition(ServoConstants.SHOOTER_TURRET_LONG.value - 0.1);

    }
    public void shoot(){
        preShooter.setPower(1);
    }

    public void outtake(){
        preShooter.setPower(-0.8);
    }

    public void emergency(){
        shooterDown.setPower(-1);
        shooterUp.setPower(-1);
    }

    public void init(){
        preShooter.setPower(-0.2);
    }


    public void stopAccelerate(){
        shooterDown.setPower(0);
        shooterUp.setPower(0);
    }
}