package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest2 extends LinearOpMode {
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
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD); //only for this robot(Broken motor)


        double servoInput=1;
        double motorInput=1;

        intakeMotor=hardwareMap.get(DcMotorEx.class,"intakeMotor");
        intakeLeft=hardwareMap.get(Servo.class,"intakeLeft");
        intakeRight=hardwareMap.get(Servo.class,"intakeRight");

        intakeRight.setDirection(Servo.Direction.REVERSE);
        intakeLeft.setDirection(Servo.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.right_bumper) {
                intakeMotor.setPower(motorInput);
                intakeLeft.setPosition(servoInput);
                intakeRight.setPosition(servoInput);
            } else {
                intakeMotor.setVelocity(0);
                intakeLeft.setPosition(0.5);
                intakeRight.setPosition(0.5);
            }
//            telemetry.addData("ServoStatusL",intakeLeft.getPower());
//            telemetry.addData("ServoStatusR",intakeRight.getPower());
//            telemetry.update();

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double powerCoefficent = 1;

            frontLeftMotor.setPower(frontLeftPower * powerCoefficent);
            backLeftMotor.setPower(backLeftPower * powerCoefficent);
            frontRightMotor.setPower(frontRightPower * powerCoefficent);
            backRightMotor.setPower(backRightPower * powerCoefficent);
        }

    }
}
