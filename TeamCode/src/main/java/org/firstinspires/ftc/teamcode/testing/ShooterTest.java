package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Test Shooter PID", group = "test")
@Config
public class ShooterTest extends LinearOpMode {
  private final Telemetry telemetry_M =
      new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
  public static boolean isPIDControl = true;
  public static double setP = 12;
  public static double setI = 0;
  public static double setD = 0;
  public static double setF = 15;
  public static double setShooterPower = 1;
  public static double setintakePower = 1;
  public static boolean isPowerMode = true;
  public static double setPreShooterPower = 1;
//  public static double shooterMinVelocity = 1400.0;
  public static double shooterVelocity = -800;

  @Override
  public void runOpMode() throws InterruptedException {
    DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
    DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
    DcMotorEx preShooter = hardwareMap.get(DcMotorEx.class, "preShooter");
    DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");

    shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

    shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

    shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

    shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

    shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    if (isPIDControl) {
      shooter1.setVelocityPIDFCoefficients(setP, setI, setD, setF);
    }

    waitForStart();

    while (opModeIsActive()) {
      if(isPowerMode){
        shooter1.setPower(setShooterPower);
      }
      else{
        shooter1.setVelocity(shooterVelocity);

      }

//      if (frontShooter.getVelocity() > shooterMinVelocity) {
      if(gamepad1.right_bumper){
        preShooter.setPower(setPreShooterPower);
        intake.setPower(setintakePower);
      }
      else{
        preShooter.setPower(0);
        intake.setPower(0);
      }

//      if (frontShooter.getVelocity() < shooterMinVelocity) {
//        //            if(gamepad1.b){
//        preShooter.setPower(0);
//        blender.setPower(0);
//        intake.setPower(0);
//      }

      telemetry_M.addData("Shooter Velocity", shooter1.getVelocity());
      telemetry_M.addData("PreShooter Velocity", preShooter.getVelocity());
      telemetry_M.update();
    }
  }
}