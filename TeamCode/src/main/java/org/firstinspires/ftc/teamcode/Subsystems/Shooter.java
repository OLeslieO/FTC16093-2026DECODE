package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotorEx shooter, preShooter;

    public Shooter(HardwareMap hardwareMap) {
        this.shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        this.preShooter = hardwareMap.get(DcMotorEx.class, "preShooter");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void accelerate(){
        shooter.setPower(0.72);
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
}