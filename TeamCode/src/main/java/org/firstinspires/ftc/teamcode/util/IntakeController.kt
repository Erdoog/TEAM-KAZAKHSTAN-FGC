package org.firstinspires.ftc.teamcode.util

import com.arcrobotics.ftclib.controller.PDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class IntakeController(
    val hardwareMap: HardwareMap,
    val telemetry: Telemetry
) {
    var bottomTarget = 1.1;
    val bottomPID = PDController(2.7, 0.0);
    val bottomIntake = Motor(hardwareMap, "bottomIntake");
    val potentiometer = hardwareMap.analogInput.get("intakeEncoder");
    val topMotor = Motor(hardwareMap, "topIntake");
    val leftDoor = hardwareMap.servo.get("leftDoor");
    val rightDoor = hardwareMap.servo.get("rightDoor");
    init {
        bottomIntake.setRunMode(Motor.RunMode.RawPower);
        bottomIntake.resetEncoder();
        topMotor.setRunMode(Motor.RunMode.RawPower);

        topMotor.inverted = true;
        topMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftDoor.position = 0.17;
        rightDoor.position = 0.92;
//        leftServo.direction = DcMotorSimple.Direction.REVERSE;
    }

    fun updateBottom(){
        val position = potentiometer.voltage;
        val power = bottomPID.calculate(1.0 * position, bottomTarget)
        bottomIntake.set(power);
        telemetry.addData("intake", position);
    }

    fun setBottom(grabbing: Boolean){
        if (!grabbing)
            bottomTarget = 1.4;
        else
            bottomTarget = 2.8;
    }

    fun setDoor(open: Boolean){
        if (open){
            leftDoor.position = 1.00;
            rightDoor.position = 0.07;
        } else {
            leftDoor.position = 0.48;
            rightDoor.position = 0.66;
        }
    }

//    fun setBottomVelocity(rate: Double){
//        bottomIntake.set(rate);
//        leftServo.power = rate;
//        rightServo.power = rate;
//    }

    fun setTopVelocity(rate: Double){
        topMotor.set(rate);
    }
}