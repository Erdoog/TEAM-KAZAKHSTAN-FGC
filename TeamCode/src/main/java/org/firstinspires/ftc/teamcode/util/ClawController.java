package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ClawController {
    public static double leftOpenState = 0.48;
    public static double leftClosedState = 0.35;
    public static double leftHiddenState = 0.20;
    public static double rightOpenState = 0.05;
    public static double rightClosedState = 0.18;
    public static double rightHiddenState = 0.38;

    private Servo leftServo;
    private Servo rightServo;
    private ElapsedTime elapsedTimer;
    private double targetState = -1.0;
    private double currentState = -1.0;
    public double velocity = 3.0;

    private void construct(Servo leftServo, Servo rightServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        this.elapsedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public ClawController(HardwareMap hardwareMap) {
        construct(hardwareMap.servo.get("leftClaw"), hardwareMap.servo.get("rightClaw"));
    }

    public void setTargetState(double targetState) {
        this.targetState = targetState;
    }

    public void update() {
        double elapsedTime = elapsedTimer.milliseconds() / 1000.0;
        elapsedTimer.reset();
        if (currentState == 1.0 && targetState == -1.0){
            currentState = 0.0;
        } else if (targetState - currentState < 0 && targetState == -1.0) {
            currentState -= Math.min(currentState - targetState, velocity * elapsedTime);
        } else {
            currentState = targetState;
        }

        if (currentState == 1.0) {
            leftServo.setPosition(leftOpenState);
            rightServo.setPosition(rightOpenState);
        } else if (0.0 <= currentState && currentState < 1.0){
            leftServo.setPosition(leftClosedState);
            rightServo.setPosition(rightClosedState);
        } else if (-1.0 < currentState && currentState < 0.0){
            leftServo.setPosition(leftClosedState);
            rightServo.setPosition(rightHiddenState);
        } else {
            leftServo.setPosition(leftHiddenState);
            rightServo.setPosition(rightHiddenState);
        }
    }
}

