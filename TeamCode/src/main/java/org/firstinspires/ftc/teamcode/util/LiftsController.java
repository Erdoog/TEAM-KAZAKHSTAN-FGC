package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LiftsController {
    public static double LIFT_KP = 0.003;
    public static double LIFT_KD = 0.0;
    public static double DIFFERENCE_KP = 0.001;
    public static double DIFFERENCE_KD = 0.0;
    public static double SPEED = 2900.0;

    private Motor leftMotor;
    private Motor rightMotor;
    private TouchSensor leftButton;
    private TouchSensor rightButton;
    private Telemetry telemetry;

    public double target = 0.0;

    private PDController leftController;
    private PDController rightController;
    private PDController differenceController;

    private ElapsedTime timer;
    private double loopTime = 0.0;
    private double forcePower = 0.0;

    public LiftsController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftMotor = new Motor(hardwareMap, "leftLift");
        rightMotor = new Motor(hardwareMap, "rightLift");
        leftButton = hardwareMap.touchSensor.get("leftButton");
        rightButton = hardwareMap.touchSensor.get("rightButton");

        leftMotor.setInverted(true);

        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);

        leftController = new PDController(LIFT_KP, LIFT_KD);
        rightController = new PDController(LIFT_KP, LIFT_KD);
        differenceController = new PDController(DIFFERENCE_KP, DIFFERENCE_KD);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public void updateTarget(double setTarget) {
        target = setTarget;
    }

    public void deviateTarget(double rate) {
        target += SPEED * loopTime * rate;
        if (target < -300) {
            target = -300.0;
        }
    }

    public void forceMove(double inputPower) {
        forcePower = inputPower;
    }

    public void update() {
        telemetry.addData("leftButton", leftButton.isPressed());
        telemetry.addData("rightButton", rightButton.isPressed());

//        if (leftButton.isPressed()) {
//            leftMotor.resetEncoder();
//        }
//        if (rightButton.isPressed()) {
//            rightMotor.resetEncoder();
//        }

        loopTime = timer.time() * 0.001;
        timer.reset();

        double leftPosition = leftMotor.getCurrentPosition() * 1.25 * 1.333;
        double rightPosition = rightMotor.getCurrentPosition() * 1.25 * 1.333;

        double equalizingPower = differenceController.calculate(leftPosition - rightPosition, 0.0);
        double leftPower = leftController.calculate(leftPosition, target) + equalizingPower;
        double rightPower = rightController.calculate(rightPosition, target) - equalizingPower;

//        if (target == 0.0 && leftMotor.getCurrentPosition() <= 30 && leftMotor.getCurrentPosition() >= -150 && !leftButton.isPressed()) {
//            leftPower -= 0.1;
//        }
//        if (target == 0.0 && rightMotor.getCurrentPosition() <= 30 && rightMotor.getCurrentPosition() >= -150 && !rightButton.isPressed()) {
//            rightPower -= 0.1;
//        }

        if (forcePower != 0.0) {
            rightPower = forcePower;
            leftPower = forcePower;
            target = 0.0;
            leftMotor.resetEncoder();
            rightMotor.resetEncoder();
        }

        leftMotor.set(leftPower);
        rightMotor.set(rightPower);

        forcePower = 0.0;
        telemetry.addData("leftPosition", leftPosition);
        telemetry.addData("rightPosition", rightPosition);
        telemetry.addData("targetPosition", target);
    }
}

