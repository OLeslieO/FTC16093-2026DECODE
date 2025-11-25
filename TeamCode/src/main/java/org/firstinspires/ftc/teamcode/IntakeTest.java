package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


@TeleOp(name = "IntakeTest")
public class IntakeTest extends LinearOpMode {


    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        double targetVelocity;
        double currentVelocity;

        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx preShooterMotor = hardwareMap.get(DcMotorEx.class, "preShooter");
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE); // only for this robot (Broken motor)
        //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();

        while (opModeIsActive()) {

            currentVelocity = shooterMotor1.getVelocity();

            if (gamepad2.dpad_up) {
                shooterMotor1.setVelocity(-1000);  //近
                shooterMotor2.setVelocity(-1000);
                targetVelocity = -1000;
                if (Math.abs(currentVelocity - targetVelocity) <= 60) {
                    preShooterMotor.setPower(1);
                } else {
                    preShooterMotor.setPower(0);
                }

            } else if (gamepad2.left_bumper) {
                shooterMotor1.setVelocity(-1150);  //中
                shooterMotor2.setVelocity(-1150);
                targetVelocity = -1150;
                if (Math.abs(currentVelocity - targetVelocity) <= 40) {
                    preShooterMotor.setPower(1);
                } else {
                    preShooterMotor.setPower(0);
                }

            } else if (gamepad2.triangle) {
                shooterMotor1.setVelocity(-1440); //远
                shooterMotor2.setVelocity(-1440);
                targetVelocity = -1440;
                if (Math.abs(currentVelocity - targetVelocity) <= 40) {
                    preShooterMotor.setPower(1);
                } else {
                    preShooterMotor.setPower(0);
                }


            } else {
                targetVelocity = -1150;
                shooterMotor1.setVelocity(-1150);
                shooterMotor2.setVelocity(-1150);
                preShooterMotor.setPower(0);
            }


            if (gamepad2.right_bumper) {
                intakeMotor.setPower(-1);  //intake
            } else if ((gamepad1.right_trigger) >= 0.5) {
                intakeMotor.setPower(1);  //outtake

            } else {
                intakeMotor.setPower(0);
            }

            telemetry.addData("currentVelocity", shooterMotor1.getVelocity());
            telemetry.addData("Difference", Math.abs(currentVelocity - targetVelocity));
            telemetry.update();

            /*
            if ((Math.abs(currentVelocity-targetVelocity) <= 100) && (gamepad1.left_bumper)) {

                preShooterMotor.setPower(1);

            } else {
                preShooterMotor.setPower(0);

            }
            */

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


