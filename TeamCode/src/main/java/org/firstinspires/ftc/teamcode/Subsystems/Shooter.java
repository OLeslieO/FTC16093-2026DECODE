package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Constants.MotorConstants;

public class Shooter {
    public DcMotorEx shooter, preShooter,shooter1,shooter2;
    double currentVelocity = shooter1.getVelocity();


    public Shooter(HardwareMap hardwareMap) {
        this.shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        this.preShooter = hardwareMap.get(DcMotorEx.class, "preShooter");
        this.shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        this.shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setVelocityPIDFCoefficients(MotorConstants.SHOOTER_P.value, MotorConstants.SHOOTER_I.value, MotorConstants.SHOOTER_D.value, MotorConstants.SHOOTER_F.value);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setVelocityPIDFCoefficients(MotorConstants.SHOOTER_P.value, MotorConstants.SHOOTER_I.value, MotorConstants.SHOOTER_D.value, MotorConstants.SHOOTER_F.value);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter2.setVelocityPIDFCoefficients(MotorConstants.SHOOTER_P.value, MotorConstants.SHOOTER_I.value, MotorConstants.SHOOTER_D.value, MotorConstants.SHOOTER_F.value);
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

    public void longShoot() { shooter1.setVelocity(1800);
        shooter2.setVelocity(1800);
        double targetVelocity = 1800;
        if (Math.abs(currentVelocity-targetVelocity) <= 80){
            preShooter.setPower(1);
        } else {
            preShooter.setPower(0);
        }}

    public void midShoot() {

    shooter1.setVelocity(1200);
        shooter2.setVelocity(1200);

        double targetVelocity = 1200;
        if (Math.abs(currentVelocity-targetVelocity) <= 80){
            preShooter.setPower(1);
        } else {
            preShooter.setPower(0);
        }}

    public void shortShoot () {
        shooter1.setVelocity(900);
        shooter2.setVelocity(900);
        double targetVelocity = 900;
        if (Math.abs(currentVelocity-targetVelocity) <= 80){
            preShooter.setPower(1);
        } else {
            preShooter.setPower(0);
        }
    }

    public void idle (){
        shooter1.setVelocity(900);
        shooter2.setVelocity(900);
        double targetVelocity = 900;

    }

    public void shooting() {
        preShooter.setPower(1);
    }
}



