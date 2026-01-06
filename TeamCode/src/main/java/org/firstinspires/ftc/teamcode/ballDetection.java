package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class ballDetection extends LinearOpMode {


    private final float[] hsvValues = {0F, 0F, 0F};


    private boolean ballDetected = false;
    private boolean purpule = false;
    private boolean green = false;

    private long ballNum = 0;

    private Queue<Integer> colorQue = new LinkedList<>();

    private List<Float> hues = new ArrayList<>();
    private ColorSensor colorSensor;
    private ColorSensor colorSensor1;
    private DistanceSensor distanceSensor;
    private DistanceSensor distanceSensor1;

    private double ballDistance = 2.6;

    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "colorSensor1");

        double dis = distanceSensor.getDistance(DistanceUnit.CM);
        double dis1 = distanceSensor1.getDistance(DistanceUnit.CM);

        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        // 转 HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);





        if (dis < ballDistance || dis1 < ballDistance) {
            hues.add(hsvValues[0]);
            if (!ballDetected) ballNum++;
            ballDetected = true;
        }

        if ((dis > ballDistance && dis1 > ballDistance) && ballDetected) {
            ballDetected = false;

            Collections.sort(hues);

            float res = hues.get(hues.size() / 2);

            if (res >= ballDetectionConstants.PURPLE_MIN_H && res <= ballDetectionConstants.PURPLE_MAX_H) {
                colorQue.offer(1);
            }
            else if (res >= ballDetectionConstants.GREEN_MIN_H && res <= ballDetectionConstants.GREEN_MAX_H){
                colorQue.offer(0);
            }
            if (colorQue.size() > 3) colorQue.poll();

            purpule = false;
            green = false;

            hues.clear();
        }

    }
}