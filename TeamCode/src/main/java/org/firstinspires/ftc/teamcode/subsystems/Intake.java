package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {
    public DcMotorEx intakeMotor;
    public Servo intakeLeft;
    public Servo intakeRight;
    private final double motorInput = 1.0;

    public Intake() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        intakeRight.setDirection(Servo.Direction.REVERSE);
        intakeLeft.setDirection(Servo.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void autoIntake() {
        intakeMotor.setPower(motorInput);
        intakeLeft.setPosition(1);
        intakeRight.setPosition(1);
    }

    public void manualIntake() {
        if (gamepad1.right_bumper) {
            intakeMotor.setPower(motorInput);
            intakeLeft.setPosition(1);
            intakeRight.setPosition(1);
        } else if (gamepad1.left_bumper) {
            intakeMotor.setPower(-motorInput);
            intakeLeft.setPosition(0);
            intakeRight.setPosition(0);
        }
        else {
            intakeMotor.setVelocity(0);
            intakeLeft.setPosition(0.5);
            intakeRight.setPosition(0.5);
        }

    }
    public void allStop() {
        intakeMotor.setVelocity(0);
        intakeLeft.setPosition(0.5);
        intakeRight.setPosition(0.5);

    }
}