package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class DebugWheels : LinearOpMode(){
    override fun runOpMode(){
        val leftFrontDrive = Motor(hardwareMap, "leftFront")
        val rightFrontDrive = Motor(hardwareMap, "rightFront")
        val leftRearDrive = Motor(hardwareMap, "leftBack")
        val rightRearDrive = Motor(hardwareMap, "rightBack")

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.left_bumper)
                leftFrontDrive.set(1.0);
            else
                leftFrontDrive.set(0.0);

            if (gamepad1.right_bumper)
                rightFrontDrive.set(1.0);
            else
                rightFrontDrive.set(0.0);

            leftRearDrive.set(gamepad1.left_trigger.toDouble());
            rightRearDrive.set(gamepad1.right_trigger.toDouble());

            telemetry.addData("lf", leftFrontDrive.currentPosition);
            telemetry.addData("rf", rightFrontDrive.currentPosition);
            telemetry.addData("lb", leftRearDrive.currentPosition);
            telemetry.addData("rb", rightRearDrive.currentPosition);

            telemetry.update();
        }
    }
}