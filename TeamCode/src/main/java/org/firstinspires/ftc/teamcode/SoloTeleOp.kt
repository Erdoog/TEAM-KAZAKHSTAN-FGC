package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.drivebase.DifferentialDrive
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.util.IntakeController
import org.firstinspires.ftc.teamcode.util.LiftsController

@TeleOp
class SoloTeleOp : OpMode() {
    var driver1: GamepadEx? = null;
    var driver2: GamepadEx? = null;
    var tankDrive: DifferentialDrive? = null
    var liftsController: LiftsController? = null
    var intakeController: IntakeController? = null;
    var feedTopIntakeTimer: ElapsedTime? = null;
    var outputIntakeTimer: ElapsedTime? = null;
    var doubleOutputIntakeTimer: ElapsedTime? = null;
    var opmodeTimer: ElapsedTime? = null;
    var elapsedTimer: ElapsedTime? = null;
//    var clawController: ClawController? = null;

    companion object  {
        var DRIVE_KP = 0.8;
        var DRIVE_KD = 0.0;
    }

    override fun init() {
        val dashTelemetry = FtcDashboard.getInstance().telemetry;
        telemetry = MultipleTelemetry(telemetry, dashTelemetry);

        driver1 = GamepadEx(gamepad1);
        driver2 = GamepadEx(gamepad2);

        val leftFrontDrive = Motor(hardwareMap, "leftFront")
        val rightFrontDrive = Motor(hardwareMap, "rightFront")
        val leftRearDrive = Motor(hardwareMap, "leftBack")
        val rightRearDrive = Motor(hardwareMap, "rightBack")

        leftFrontDrive.setRunMode(Motor.RunMode.VelocityControl);
        rightFrontDrive.setRunMode(Motor.RunMode.VelocityControl);
        leftRearDrive.setRunMode(Motor.RunMode.VelocityControl);
        rightRearDrive.setRunMode(Motor.RunMode.VelocityControl);

        leftFrontDrive .setVeloCoefficients(DRIVE_KP, 0.0, DRIVE_KD);
        leftRearDrive  .setVeloCoefficients(DRIVE_KP, 0.0, DRIVE_KD);
        rightFrontDrive.setVeloCoefficients(DRIVE_KP, 0.0, DRIVE_KD);
        rightRearDrive .setVeloCoefficients(DRIVE_KP, 0.0, DRIVE_KD);

        leftFrontDrive.ACHIEVABLE_MAX_TICKS_PER_SECOND = 3200.0;
        rightFrontDrive.ACHIEVABLE_MAX_TICKS_PER_SECOND = 3200.0;
        leftRearDrive.ACHIEVABLE_MAX_TICKS_PER_SECOND = 3200.0;
        rightRearDrive.ACHIEVABLE_MAX_TICKS_PER_SECOND = 3200.0;

        rightFrontDrive.inverted = true
//        leftFrontDrive.inverted = true
//        rightRearDrive.inverted = true
//        leftRearDrive.inverted = true

        val leftDriveGroup = MotorGroup(leftFrontDrive, leftRearDrive)
        val rightDriveGroup = MotorGroup(rightFrontDrive, rightRearDrive)

        tankDrive = DifferentialDrive(leftDriveGroup, rightDriveGroup);
        liftsController = LiftsController(
            hardwareMap,
            telemetry
        );
        intakeController = IntakeController(hardwareMap, telemetry);
//        clawController = ClawController(hardwareMap);

        opmodeTimer = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTimer = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        feedTopIntakeTimer = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        outputIntakeTimer = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        doubleOutputIntakeTimer = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    override fun start() {
        opmodeTimer!!.reset();
        elapsedTimer!!.reset();
        feedTopIntakeTimer!!.reset();

        super.start()
    }

    var doorState = false;

    override fun loop() {
        var elapsedTime = elapsedTimer!!.milliseconds() / 1000.0;
        elapsedTimer!!.reset();
        var forwardInput = driver1!!.leftY;
        var leftInput = driver1!!.rightX;
//        if (driver1!!.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0){
//            forwardInput *= 0.47;
//            leftInput *= 0.47;
//        }
        forwardInput *= 1 - Math.pow(driver1!!.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), 3.0) * 0.6;
        leftInput *= 1 - Math.pow(driver1!!.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), 3.0) * 0.6;

        arcadeDrive(forwardInput, leftInput, elapsedTime);

        var liftInput = 0.0
        if (driver1!!.getButton(GamepadKeys.Button.RIGHT_BUMPER))
            liftInput += 1.0;
        if (driver1!!.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0)
            liftInput -= 1.0;

        liftsController!!.deviateTarget(liftInput);

//        if (driver1!!.getButton(GamepadKeys.Button.Y))
//            liftsController!!.target = 13800.0;
//        if (driver1!!.getButton(GamepadKeys.Button.X))
//            liftsController!!.target = 10000.0;
//        if (driver1!!.getButton(GamepadKeys.Button.B))
//            liftsController!!.target = 2500.0;
//        if (driver1!!.getButton(GamepadKeys.Button.A))
//            liftsController!!.target = 0.0;

        if (driver1!!.getButton(GamepadKeys.Button.Y))
            liftsController!!.target = 14000.0;
        if (driver1!!.getButton(GamepadKeys.Button.X))
            liftsController!!.target = 12200.0;
//        if (driver1!!.getButton(GamepadKeys.Button.B))
//            liftsController!!.target = 1600.0;
        if (driver1!!.getButton(GamepadKeys.Button.A))
            liftsController!!.target = 0.0;

        if (liftsController!!.target > 14500.0)
            liftsController!!.target = 14500.0;

        liftsController!!.update();

        if (driver1!!.getButton(GamepadKeys.Button.DPAD_UP))
            intakeController!!.setBottom(true);
        if (driver1!!.getButton(GamepadKeys.Button.DPAD_DOWN))
            intakeController!!.setBottom(false);

        var topIntakeInput = 0.0;
        if (driver2!!.getButton(GamepadKeys.Button.RIGHT_BUMPER) || driver1!!.getButton(GamepadKeys.Button.LEFT_BUMPER))
            topIntakeInput += 1;
        if (driver2!!.getButton(GamepadKeys.Button.LEFT_BUMPER))
            topIntakeInput -= 1;

        if (driver1!!.wasJustPressed(GamepadKeys.Button.B)){
            doorState = !doorState;
            if (doorState)
                intakeController!!.setDoor(true);
            else
                intakeController!!.setDoor(false);
        }


//        if (opmodeTimer!!.milliseconds() < 800)
//            topIntakeInput = -1.0;

//        if (driver2!!.wasJustPressed(GamepadKeys.Button.X) && opmodeTimer!!.milliseconds() > 550 && topIntakeInput == 0.0)
//            feedTopIntakeTimer!!.reset();
//        if (feedTopIntakeTimer!!.milliseconds() < 550 && opmodeTimer!!.milliseconds() > 550)
//            topIntakeInput = 1.0;
//
//        if (driver2!!.wasJustPressed(GamepadKeys.Button.A) && opmodeTimer!!.milliseconds() > 800 && topIntakeInput == 0.0)
//            outputIntakeTimer!!.reset();
//        if (outputIntakeTimer!!.milliseconds() < 800 && opmodeTimer!!.milliseconds() > 800)
//            topIntakeInput = 1.0;
//
//        if (driver2!!.wasJustPressed(GamepadKeys.Button.Y) && opmodeTimer!!.milliseconds() > 1700 && topIntakeInput == 0.0)
//            doubleOutputIntakeTimer!!.reset();
//        if (doubleOutputIntakeTimer!!.milliseconds() < 1700 && opmodeTimer!!.milliseconds() > 1700)
//            topIntakeInput = 1.0;
//
//        if (driver2!!.wasJustPressed(GamepadKeys.Button.Y)){
//            clawController!!.setTargetState(1.0);
//            liftsController!!.target = 1800.0;
//        }
//        if (driver2!!.wasJustPressed(GamepadKeys.Button.X) || driver2!!.wasJustPressed(GamepadKeys.Button.B)){
//            clawController!!.setTargetState(0.0);
//            liftsController!!.target = 1600.0;
//        }
//        if (driver2!!.wasJustPressed(GamepadKeys.Button.A)){
//            clawController!!.setTargetState(-1.0);
//            liftsController!!.target = 0.0;
//        }

        intakeController!!.updateBottom();
        intakeController!!.setTopVelocity(topIntakeInput);
//        clawController!!.update();

        driver1!!.readButtons()
        driver2!!.readButtons()
    }

    fun arcadeDrive(forwardInput: Double, turnInput: Double, elapsedTime: Double){
        tankDrive!!.arcadeDrive(forwardInput, turnInput);
    }
}