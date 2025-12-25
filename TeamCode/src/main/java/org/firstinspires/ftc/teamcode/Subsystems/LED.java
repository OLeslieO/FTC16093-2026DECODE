package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Constants.MotorConstants;

public class LED {
    public Servo indicatorLight;


    public LED(HardwareMap hardwareMap) {
        this.indicatorLight = hardwareMap.get(Servo.class, "indicatorLight");


    }

    public void setBlue() {
        indicatorLight.setPosition(0.6);
    }
    public void setRed() {
        indicatorLight.setPosition(0.277);
    }


    public void setNone(){
        indicatorLight.setPosition(0);
    }



}