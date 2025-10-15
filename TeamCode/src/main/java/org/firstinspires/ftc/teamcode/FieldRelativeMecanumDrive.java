package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Relative Mecanum Drive", group = "Main")
public class FieldRelativeMecanumDrive extends CommandOpMode {

    // Declare motors
    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    
    // Declare IMU
    private IMU imu;
    
    // Gamepad wrapper
    private GamepadEx driverGamepad;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        // Initialize motors with hardware mapping matching existing configuration
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFrontMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftBackMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFrontMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "rightBackMotor");

        // Set motor directions to match existing configuration
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
        
        // Initialize gamepad
        driverGamepad = new GamepadEx(gamepad1);
        
        telemetry.addLine("Field Relative Mecanum Drive Initialized");
        telemetry.addLine("Press A to reset Yaw");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        
        // Reset IMU yaw when A button is pressed
        if (driverGamepad.getButton(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A)) {
            imu.resetYaw();
        }
        
        // Get joystick values
        double forward = -driverGamepad.getLeftY();  // Forward/Backward
        double right = driverGamepad.getLeftX();     // Left/Right
        double rotate = driverGamepad.getRightX();   // Rotation
        
        // Drive with field relative control
        driveFieldRelative(forward, right, rotate);
        
        // Update telemetry
        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Right", "%.2f", right);
        telemetry.addData("Rotate", "%.2f", rotate);
        telemetry.addData("Yaw", "%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.update();
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

    // Basic mecanum drive method
    public void drive(double forward, double right, double rotate) {
        // Calculate power for each wheel
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;

        // Normalize powers to [-1, 1] range
        double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower / maxPower);
        frontRightMotor.setPower(frontRightPower / maxPower);
        backLeftMotor.setPower(backLeftPower / maxPower);
        backRightMotor.setPower(backRightPower / maxPower);
    }
}