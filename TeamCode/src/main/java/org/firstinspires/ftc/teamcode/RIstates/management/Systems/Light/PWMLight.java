package org.firstinspires.ftc.teamcode.RIstates.management.Systems.Light;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class PWMLight {
    Servo light;
    public PWMLight(HardwareMap hardwareMap) {
        light = hardwareMap.get(Servo.class, "led");
    }

    public void off() {
        light.setPosition(0.0);
    }

    public void red() {
        light.setPosition(0.277);
    }

    public void orange() {
        light.setPosition(0.333);
    }

    public void yellow() {
        light.setPosition(0.388);
    }

    public void sage() {
        light.setPosition(0.444);
    }

    public void green() {
        light.setPosition(0.500);
    }

    public void azure() {
        light.setPosition(0.555);
    }

    public void blue() {
        light.setPosition(0.611);
    }

    public void indigo() {
        light.setPosition(0.666);
    }

    public void violet() {
        light.setPosition(0.722);
    }

    public void white() {
        light.setPosition(1.0);
    }

}
