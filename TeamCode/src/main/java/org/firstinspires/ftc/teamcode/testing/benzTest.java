package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "benz test")
@Config
public class benzTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static boolean read_only = false;
    public static boolean reverse = false;
    public static double motor_pow1 = 0.2;

    public static String motor_name1 = "benzMotor" ;
    private DcMotorEx motor0;



    @Override
    public void runOpMode() {


        motor0 = (DcMotorEx) hardwareMap.get(DcMotorEx.class, motor_name1);


        if (reverse){
            motor0.setDirection(DcMotorEx.Direction.REVERSE);
        }
        waitForStart();
        while (opModeIsActive()) {
            if (!read_only) {
                motor0.setPower(motor_pow1);


                // Show the position of the motor on telemetry
                telemetry.addData("Encoder Position", motor0.getCurrentPosition());
                

                telemetry_M.addData("leftFront", motor0.getPower());

                telemetry_M.update();
            }
        }
    }
}