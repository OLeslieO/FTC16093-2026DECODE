package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrive extends SubsystemBase{
    public DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    public Telemetry telemetry;


    public MecanumDrive(Telemetry telemetry, final HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.frontLeftMotor = hardwareMap.get(DcMotorEx.class,"leftFrontMotor");
        this.backLeftMotor = hardwareMap.get(DcMotorEx.class,"leftBackMotor");
        this.frontRightMotor = hardwareMap.get(DcMotorEx.class,"rightFrontMotor");
        this.backRightMotor = hardwareMap.get(DcMotorEx.class,"rightBackMotor");
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

    }
    public void drive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        double powerCoefficient = 1.0;

        frontLeftMotor.setPower(frontLeftPower * powerCoefficient);
        backLeftMotor.setPower(backLeftPower * powerCoefficient);
        frontRightMotor.setPower(frontRightPower * powerCoefficient);
        backRightMotor.setPower(backRightPower * powerCoefficient);
    }

}