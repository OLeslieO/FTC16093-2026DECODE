package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "IntakeTest")
public class IntakeTest extends LinearOpMode {
    private DcMotorEx intakeMotor;
    private Servo intakeLeft;
    private Servo intakeRight;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"leftFrontMotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"leftBackMotor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"rightFrontMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"rightBackMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD); // only for this robot (Broken motor)

        double motorInput = 1.0;

        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        intakeLeft = hardwareMap.get(Servo.class,"intakeLeft");
        intakeRight = hardwareMap.get(Servo.class,"intakeRight");

        intakeRight.setDirection(Servo.Direction.REVERSE);
        intakeLeft.setDirection(Servo.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while(opModeIsActive()) {
            // ---- Intake 控制 ----
            if (gamepad1.right_bumper) {
                // 吸球：电机正转
                intakeMotor.setPower(motorInput);

                // 舵机快速抖动（往复角度）
                intakeLeft.setPosition(0.3);
                intakeRight.setPosition(0.3);
                sleep(50);

                intakeLeft.setPosition(0.7);
                intakeRight.setPosition(0.7);
                sleep(50);

            } else if (gamepad1.left_bumper) {
                // 吐球：电机反转
                intakeMotor.setPower(-motorInput);

                // 舵机反向抖动
                intakeLeft.setPosition(0.7);
                intakeRight.setPosition(0.7);
                sleep(50);

                intakeLeft.setPosition(0.3);
                intakeRight.setPosition(0.3);
                sleep(50);

            } else {
                // 停止
                intakeMotor.setPower(0);
                intakeLeft.setPosition(0.5);  // 回到中位
                intakeRight.setPosition(0.5);
            }

            // ---- Mecanum 驱动 ----
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
}
