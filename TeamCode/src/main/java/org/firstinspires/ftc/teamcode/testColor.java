package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Timer;

@TeleOp(name="testColor")
public class testColor extends LinearOpMode {

    //green
    public static int GREEN_MIN_H = 150;
    public static int GREEN_MAX_H = 170;
    public static int GREEN_MIN_S = 45;
    public static int GREEN_MIN_V = 48;

    // purple
    public static int PURPLE_MIN_H = 205;
    public static int PURPLE_MAX_H = 240;
    public static int PURPLE_MIN_S = 30;
    //public static int PURPLE_MIN_V = 1000;

    @Override
    public void runOpMode() {

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class,"leftRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class,"rightRear");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class,"intake");


        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        ArrayList<Integer> colorRecord = new ArrayList<>();
        boolean inZone = false;
        int ballCount = 0;

        waitForStart();

        while (opModeIsActive()) {


            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;

            double a = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
            leftFront.setPower((y + x + rx) / a);
            leftRear.setPower((y - x + rx) / a);
            rightFront.setPower((y - x - rx) / a);
            rightRear.setPower((y + x - rx) / a);

            // =raw rgb
            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();

            // hsv
            float[] hsv = new float[3];
            Color.RGBToHSV(r, g, b, hsv);

            float H = hsv[0];
            float S = hsv[1] * 100;  // 转成百分比
            float V = hsv[2] * 100;


            boolean isGreen =
                    (H >= GREEN_MIN_H && H <= GREEN_MAX_H &&
                            S >= GREEN_MIN_S &&
                            V >= GREEN_MIN_V);

            boolean isPurple =
                    (H >= PURPLE_MIN_H && H <= PURPLE_MAX_H &&
                            S >= PURPLE_MIN_S );

            double distance = ((DistanceSensor)colorSensor).getDistance(DistanceUnit.CM);

            if (distance <= 2.7 && !inZone) {

                if (isPurple) {
                    colorRecord.add(1);
                    ballCount++;
                    inZone = true;
                }
                else if (isGreen) {
                    colorRecord.add(0);
                    ballCount++;
                    inZone = true;
                }
                // 如果没检测到，不设 inZone，继续检测
            }
            if (distance > 2.7) {
                inZone = false;
            }
            if(ballCount >= 4){
                intake.setPower(0.9);
                sleep(240);
                intake.setPower(-0.55);
                ballCount = 0;
            } else {
                intake.setPower(-0.55);
            }


            telemetry.addData("RGB", "R:%d G:%d B:%d", r, g, b);
            telemetry.addData("HSV", "H:%.1f S:%.1f V:%.1f", H, S, V);
            telemetry.addData("Distance", distance);

            telemetry.addLine("--- Detection ---");
            telemetry.addData("isGreen", isGreen);
            telemetry.addData("isPurple", isPurple);

            telemetry.addLine("--- Record ---");
            telemetry.addData("List", colorRecord.toString());
            telemetry.addData("ballCount",ballCount);
            telemetry.addData("InZone", inZone);

            telemetry.update();
        }
    }
}
