package org.firstinspires.ftc.teamcode.opmodes

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem
import kotlin.math.absoluteValue
import kotlin.math.pow

@TeleOp(name="TeleOpCore (Kotlin)")
class TeleOpCore: OpMode() {

    lateinit var robot: ExtThinBot
    lateinit var gamepad1Ex: GamepadEx
    lateinit var gamepad2Ex: GamepadEx

    private var reverse = false/* by DashboardVar(false, "reverse", this::class)*/
    private var speed = 1/*by DashboardVar(1, "speed", this::class) {it in 1..3}*/
    private var speedMax: Double = 4.0
    private var maxRpm = 435
    private var cubicEnable = false
    private var fod = false
    private var zeroAngle = 0.0
    private var lastTimeRead = 0.0

    private var defaultCarouselSpeed = -0.50/*by DashboardVar(-0.25, "defaultCarouselSpeed", this::class)*/
    private var maxCarouselSpeed = 0.8/*by DashboardVar(0.6, "defaultCarouselSpeed", this::class) {it in 0.0..1.0}*/

    private var depositLiftPowerAuto = 0.5/*by DashboardVar(0.5, "depositLiftPowerAuto", this::class) { it.absoluteValue <= 1.0}*/

    private var defaultWebcamPosition = 1.0/*by DashboardVar(1.0, "defaultWebcamPosition", this::class) { it in 0.0..1.0 }*/

    private lateinit var elapsedTime: ElapsedTime

    private val gamepad1CanControlAccessories: Boolean
        get() = gamepad1.left_bumper && gamepad1.right_bumper

    private val gamepad2CanControlExtras: Boolean
        get() = gamepad2.right_bumper

    override fun init() {
        robot = ExtThinBot(hardwareMap)
        robot.holonomic.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        robot.carouselMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        gamepad1Ex = GamepadEx(gamepad1)
        gamepad2Ex = GamepadEx(gamepad2)
        elapsedTime = ElapsedTime()
    }

    override fun loop() {

        gamepad1Ex.readButtons()
        gamepad2Ex.readButtons()

        // Control deposit servo
        when {
            gamepad2.a || gamepad1CanControlAccessories && gamepad1.a -> robot.fullIntakeSystem.depositServoIsExtended = true
            gamepad2.b || gamepad1CanControlAccessories && gamepad1.b -> robot.fullIntakeSystem.depositServoIsExtended = false
        }

        // Control both intake motors
        val dualIntakeMotorPower = when {
            gamepad2.right_trigger > 0.05 -> -gamepad2.right_trigger.toDouble()
            gamepad2.left_trigger > 0.05 && !gamepad2CanControlExtras -> gamepad2.left_trigger.toDouble() * 0.5
            gamepad1.right_trigger > 0.05 -> -gamepad1.right_trigger.toDouble()
            gamepad1.left_trigger > 0.05 && !gamepad1CanControlAccessories -> gamepad1.left_trigger.toDouble() * 0.5
            else -> 0.0
        }
        if (dualIntakeMotorPower.absoluteValue > 0)
            robot.fullIntakeSystem.intakeManual(dualIntakeMotorPower)
        else if (!gamepad2.right_bumper)
            robot.fullIntakeSystem.intakeManual(gamepad2.left_stick_y.toDouble(), gamepad2.right_stick_y.toDouble())
        else
            robot.fullIntakeSystem.intakeManual(0.0)

        // Control carousel motor
        robot.carouselMotor.power = when {
            gamepad2CanControlExtras && gamepad2.left_stick_y.absoluteValue > 0.05 ->
                gamepad2.left_stick_y.toDouble().coerceIn(-maxCarouselSpeed, maxCarouselSpeed)
            gamepad2CanControlExtras && gamepad2.left_trigger > 0.05 -> gamepad2.left_trigger * defaultCarouselSpeed
            gamepad1CanControlAccessories && gamepad1.left_trigger > 0.05 -> gamepad1.left_trigger * defaultCarouselSpeed
            gamepad2.left_bumper -> defaultCarouselSpeed
            else -> 0.0
        }

        // Adjust default carousel speed
        when {
            (gamepad2CanControlExtras && gamepad2Ex.wasJustPressed(DPAD_UP))
                    -> defaultCarouselSpeed += 0.05
            (gamepad2CanControlExtras && gamepad2Ex.wasJustPressed(DPAD_DOWN))
                   -> defaultCarouselSpeed -= 0.05
            (gamepad2CanControlExtras && gamepad2Ex.wasJustPressed(X))
                    || (gamepad1CanControlAccessories && gamepad1Ex.wasJustPressed(X)) -> defaultCarouselSpeed *= -1.0
            (gamepad2CanControlExtras && gamepad2Ex.wasJustPressed(DPAD_LEFT)) ->
                robot.carouselMotor.zeroPowerBehavior =
                        if (robot.carouselMotor.zeroPowerBehavior == DcMotor.ZeroPowerBehavior.FLOAT) DcMotor.ZeroPowerBehavior.BRAKE
                        else DcMotor.ZeroPowerBehavior.FLOAT
        }

        // Control deposit lift
        val depositLiftPower = if (gamepad2CanControlExtras) gamepad2.right_stick_y.toDouble()*0.5 else 0.0
        if (robot.depositLiftMotor.mode == DcMotor.RunMode.RUN_TO_POSITION) {
            if (gamepad2.right_stick_y.absoluteValue > 0) robot.fullIntakeSystem.depositLiftManual(0.0)
        } else {
            robot.fullIntakeSystem.depositLiftManual(depositLiftPower)
        }

        if (!gamepad2.right_bumper) {
            when {
                !gamepad2CanControlExtras && gamepad2.dpad_down || gamepad1CanControlAccessories && gamepad1.dpad_down ->
                    robot.fullIntakeSystem.depositLiftAuto(FullIntakeSystem.DepositLiftPosition.LOW, depositLiftPowerAuto)
                !gamepad2CanControlExtras && gamepad2.dpad_right || gamepad1CanControlAccessories && gamepad1.dpad_right ->
                    robot.fullIntakeSystem.depositLiftAuto(FullIntakeSystem.DepositLiftPosition.MIDDLE, depositLiftPowerAuto)
                !gamepad2CanControlExtras && gamepad2.dpad_up || gamepad1CanControlAccessories && gamepad1.dpad_up ->
                    robot.fullIntakeSystem.depositLiftAuto(FullIntakeSystem.DepositLiftPosition.HIGH, depositLiftPowerAuto)
            }
        }

        if (gamepad2.y || gamepad1CanControlAccessories && gamepad1.y) robot.fullIntakeSystem.resetDepositZero()

        // Control webcam servo
        robot.webcamServo.position = defaultWebcamPosition

        if (!gamepad1CanControlAccessories) {
            // Adjust drivetrain speed
            when {
                gamepad1Ex.wasJustPressed(DPAD_UP) -> if (speed < speedMax) speed++
                gamepad1Ex.wasJustPressed(DPAD_DOWN) -> if (speed > 1) speed--
                gamepad1Ex.wasJustPressed(DPAD_LEFT) -> cubicEnable = !cubicEnable
            }

            // Reverse drivetrain, and cubic enable
            when {
                gamepad1.a -> { reverse = true; fod = false }
                gamepad1.b -> { reverse = false; fod = false }
                gamepad1Ex.wasJustPressed(Y) -> { reverse = !reverse; fod = false }
                gamepad1Ex.wasJustPressed(X) -> fod = true
                gamepad1.x && gamepad1.start -> zeroAngle = robot.imuControllerC.getHeading()
            }
        }

        val vertical = -gamepad1.left_stick_y.toDouble().pow(if (cubicEnable) 3 else 1) * (speed/speedMax) * (if (reverse) 1 else -1)
        val horizontal = gamepad1.left_stick_x.toDouble().pow(if (cubicEnable) 3 else 1) * (speed/speedMax) * (if (reverse) 1 else -1)
        val pivot = gamepad1.right_stick_x.toDouble().pow(if (cubicEnable) 3 else 1) * (speed/speedMax)

        robot.holonomic.runWithoutEncoderVectored(horizontal, vertical, pivot,
                if (fod) zeroAngle - robot.imuControllerC.getHeading() else 0.0)

        val currentTime = elapsedTime.milliseconds()
        telemetry.addData("Time Δ (ms)", currentTime - lastTimeRead)
        telemetry.addLine()
        lastTimeRead = currentTime

        telemetry.addData("Drivetrain speed", "$speed out of $speedMax")
        telemetry.addData("Drivetrain speed adj", speed/speedMax)
        telemetry.addData("Drivetrain max rpm", maxRpm * (speed/speedMax))
        telemetry.addData("Cubic enable", cubicEnable)
        telemetry.addLine()
        telemetry.addData("Carousel speed", defaultCarouselSpeed)
        telemetry.addData("Carousel side", if (defaultCarouselSpeed < 0) "BLUE" else "RED")
        telemetry.addData("Carousel stop behavior", robot.carouselMotor.zeroPowerBehavior)
        telemetry.addLine()
        telemetry.addData("Deposit lift power", depositLiftPower)
        telemetry.addData("Deposit lift position", robot.depositLiftMotor.currentPosition)
        telemetry.addData("Deposit lift mode", robot.depositLiftMotor.mode)
        telemetry.addData("Combined intake motor power", dualIntakeMotorPower)
        telemetry.addLine()
        telemetry.addData("Vertical", vertical)
        telemetry.addData("Horizontal", horizontal)
        telemetry.addData("Pivot", pivot)
        telemetry.addLine()
        telemetry.addData("frontRightMotor", robot.frontRightMotor.power)
        telemetry.addData("backRightMotor", robot.backRightMotor.power)
        telemetry.addData("frontLeftMotor", robot.frontLeftMotor.power)
        telemetry.addData("backLeftMotor", robot.backLeftMotor.power)
        telemetry.addLine()
        telemetry.addData("gamepad2.right_trigger", gamepad2.right_trigger)
        telemetry.addData("gamepad2.left_trigger", gamepad2.left_trigger)
        telemetry.addData("gamepad1.right_trigger", gamepad1.right_trigger)
        telemetry.addData("gamepad1.left_trigger", gamepad1.left_trigger)

        robot.fullIntakeSystem.update()
    }
}