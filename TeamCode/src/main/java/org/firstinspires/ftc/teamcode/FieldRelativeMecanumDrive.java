package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Relative Mecanum Drive", group = "Main")
public class FieldRelativeMecanumDrive extends LinearOpMode {

    // Declare motors
    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    
    // Declare IMU
    private IMU imu;

    @Override
    public void runOpMode() {
        // Initialize motors with hardware mapping matching Chasis.java
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFrontMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftBackMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFrontMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "rightBackMotor");

        // Set motor directions to match Chasis.java
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        
        // Configure IMU orientation (adjust as needed for your robot)
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = 
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = 
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        
        telemetry.addLine("Field Relative Mecanum Drive Initialized");
        telemetry.addLine("Press A to reset Yaw");
        telemetry.update();
        
        waitForStart();

        while (opModeIsActive()) {
            // Reset IMU yaw when A button is pressed
            if (gamepad1.a) {
                imu.resetYaw();
            }
            
            // Get joystick values
            double forward = -gamepad1.left_stick_y;  // Forward/Backward
            double right = gamepad1.left_stick_x;     // Left/Right
            double rotate = -gamepad1.right_stick_x;   // Rotation
            
            // Drive with field relative control
            driveFieldRelative(forward, right, rotate);
            
            // Update telemetry
            telemetry.addData("Forward", "%.2f", forward);
            telemetry.addData("Right", "%.2f", right);
            telemetry.addData("Rotate", "%.2f", rotate);
            telemetry.addData("Yaw", "%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.update();
        }
    }

    // Field relative drive method
    private void driveFieldRelative(double forward, double right, double rotate) {
        // Convert direction to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Rotate angle by the robot's current yaw
        theta = AngleUnit.normalizeRadians(theta - 
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Convert back to cartesian coordinates
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Drive with robot relative values
        drive(newForward, newRight, rotate);
    }

    // Basic mecanum drive method with power normalization like Chasis.java
    public void drive(double forward, double right, double rotate) {
        // Calculate power for each wheel
        double frontLeftPower = forward + right + rotate;
        double backLeftPower = forward - right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;

        // Normalize powers using the method from Chasis.java
        double denominator = Math.max(Math.abs(forward) + Math.abs(right) + Math.abs(rotate), 1);
        
        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower / denominator);
        backLeftMotor.setPower(backLeftPower / denominator);
        frontRightMotor.setPower(frontRightPower / denominator);
        backRightMotor.setPower(backRightPower / denominator);
    }
}