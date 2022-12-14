package org.firstinspires.ftc.teamcode.opmodes

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.Companion.persistingAllianceColor
import org.firstinspires.ftc.teamcode.library.functions.ToggleButtonWatcher
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot
import org.firstinspires.ftc.teamcode.library.robot.systems.lt.TseGrabber
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem
import kotlin.math.absoluteValue
import kotlin.math.pow

@TeleOp(name="TeleOpCore (Kotlin)")
class TeleOpCore: OpMode() {

    lateinit var robot: ExtThinBot
    lateinit var gamepad1Ex: GamepadEx
    lateinit var gamepad2Ex: GamepadEx

    private var reverse = false/* by DashboardVar(false, "reverse", this::class)*/
    private var speed = 3/*by DashboardVar(1, "speed", this::class) {it in 1..3}*/
    private var speedMax: Double = 5.0
    private var maxRpm = 435
    private var cubicEnable = false
    private var fod = false
    private var zeroAngle = 0.0
    private var lastTimeRead = 0.0

    private var defaultCarouselSpeed = when (persistingAllianceColor) {
        AllianceColor.BLUE -> -0.50
        AllianceColor.RED -> 0.50
    } /*by DashboardVar(-0.25, "defaultCarouselSpeed", this::class)*/
    private var maxCarouselSpeed = 0.8/*by DashboardVar(0.6, "defaultCarouselSpeed", this::class) {it in 0.0..1.0}*/

    private var depositLiftPowerAuto = 0.5/*by DashboardVar(0.5, "depositLiftPowerAuto", this::class) { it.absoluteValue <= 1.0}*/

    private var defaultWebcamPosition = 0.1/*by DashboardVar(1.0, "defaultWebcamPosition", this::class) { it in 0.0..1.0 }*/

    private lateinit var elapsedTime: ElapsedTime

    private var initialTilt: Double = -1.57
    private val tiltThreshhold: Double = 0.1

    private val orientation get() = robot.imuControllerC.imuA.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS)
    private val currentTilt get() = robot.imuControllerC.imuA.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).secondAngle.toDouble()

    private val gamepad1CanControlAccessories: Boolean
        get() = gamepad1.left_bumper && gamepad1.right_bumper

    private val gamepad2CanControlExtras: Boolean
        get() = gamepad2.right_bumper

    private lateinit var toggleGamepad1TouchpadPress: ToggleButtonWatcher
    private lateinit var toggleGamepad2TouchpadPress: ToggleButtonWatcher

    override fun init() {
        robot = ExtThinBot(hardwareMap)
        robot.holonomic.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        robot.carouselMotorSystem.carouselMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        robot.carouselMotorSystem.carouselMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        robot.odometryLift.raise()
        robot.tseGrabber.state = TseGrabber.TseGrabberState.STORAGE

        gamepad1Ex = GamepadEx(gamepad1)
        gamepad2Ex = GamepadEx(gamepad2)
        elapsedTime = ElapsedTime()

        initialTilt = currentTilt

        toggleGamepad1TouchpadPress = ToggleButtonWatcher { gamepad1.touchpad }
        toggleGamepad2TouchpadPress = ToggleButtonWatcher { gamepad2.touchpad }
    }

    override fun loop() {


        if ((currentTilt - initialTilt).absoluteValue >= tiltThreshhold) {
            gamepad1.rumble(1.0,1.0,1000)
        }

        gamepad1Ex.readButtons()
        gamepad2Ex.readButtons()
        toggleGamepad1TouchpadPress()
        toggleGamepad2TouchpadPress()

        // Control deposit servo
        when {
            (gamepad2.a && !gamepad2CanControlExtras) || gamepad1CanControlAccessories && gamepad1.a -> robot.fullIntakeSystem.depositServoIsExtended = true
            (gamepad2.b && !gamepad2CanControlExtras) || gamepad1CanControlAccessories && gamepad1.b -> robot.fullIntakeSystem.depositServoIsExtended = false
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
        robot.carouselMotorSystem.carouselMotor.power = when {
//            gamepad2CanControlExtras && gamepad2.left_stick_y.absoluteValue > 0.05 ->
//                gamepad2.left_stick_y.toDouble().coerceIn(-maxCarouselSpeed, maxCarouselSpeed)
//            gamepad2CanControlExtras && gamepad2.left_trigger > 0.05 -> gamepad2.left_trigger * defaultCarouselSpeed
//            gamepad1CanControlAccessories && gamepad1.left_trigger > 0.05 -> gamepad1.left_trigger * defaultCarouselSpeed
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
                    || (gamepad1CanControlAccessories && gamepad1Ex.wasJustPressed(X)) -> robot.carouselMotorSystem.start()
            (gamepad2CanControlExtras && gamepad2Ex.wasJustPressed(DPAD_LEFT)) ->
                robot.carouselMotorSystem.carouselMotor.zeroPowerBehavior =
                        if (robot.carouselMotorSystem.carouselMotor.zeroPowerBehavior == DcMotor.ZeroPowerBehavior.FLOAT) DcMotor.ZeroPowerBehavior.BRAKE
                        else DcMotor.ZeroPowerBehavior.FLOAT
            (gamepad2Ex.wasJustPressed(LEFT_STICK_BUTTON)) -> {
                defaultCarouselSpeed *= -1
                persistingAllianceColor = if (persistingAllianceColor == AllianceColor.BLUE) AllianceColor.RED else AllianceColor.BLUE
            }
        }
        if (!gamepad2.left_bumper) robot.carouselMotorSystem.run()

        // Control deposit lift
        val depositLiftPower = if (gamepad2CanControlExtras) gamepad2.right_stick_y.toDouble()*0.5 else 0.0
        if (robot.depositLiftMotor.mode == DcMotor.RunMode.RUN_TO_POSITION) {
            if (gamepad2.right_stick_y.absoluteValue > 0) robot.fullIntakeSystem.depositLiftManual(0.0)
        } else if (gamepad2.x && !gamepad2CanControlExtras) {
            robot.fullIntakeSystem.depositLiftManual(-0.10)
        }
        else {
            robot.fullIntakeSystem.depositLiftManual(depositLiftPower)
        }

        if (!gamepad2.right_bumper) {
            when {
                !gamepad2CanControlExtras && gamepad2.dpad_down || gamepad1CanControlAccessories && gamepad1.dpad_down ->
                    robot.fullIntakeSystem.depositLiftAuto(FullIntakeSystem.DepositLiftPosition.LOW, depositLiftPowerAuto)
                !gamepad2CanControlExtras && gamepad2.dpad_right || gamepad1CanControlAccessories && gamepad1.dpad_right ->
                    robot.fullIntakeSystem.depositLiftAuto(FullIntakeSystem.DepositLiftPosition.MIDDLE, depositLiftPowerAuto)
                !gamepad2CanControlExtras && gamepad2.dpad_left || gamepad1CanControlAccessories && gamepad1.dpad_left ->
                    robot.fullIntakeSystem.depositLiftAuto(FullIntakeSystem.DepositLiftPosition.TSE, depositLiftPowerAuto)
                !gamepad2CanControlExtras && gamepad2.dpad_up || gamepad1CanControlAccessories && gamepad1.dpad_up ->
                    robot.fullIntakeSystem.depositLiftAuto(FullIntakeSystem.DepositLiftPosition.HIGH, depositLiftPowerAuto)
            }
        }

        // TSE Grabber
        if (gamepad2CanControlExtras) {
            when {
                gamepad2Ex.wasJustPressed(A) -> robot.tseGrabber.nextState()
                gamepad2Ex.wasJustPressed(B) -> robot.tseGrabber.prevState()
                gamepad2Ex.wasJustPressed(Y) -> robot.tseGrabber.move(pivot = TseGrabber.PivotPosition.RELEASE_HIGHER)
            }
        }
        if (toggleGamepad2TouchpadPress.lastState) {
            when {
                gamepad2.touchpad_finger_1_x > 0.3 -> robot.tseGrabber.nextState()
                gamepad2.touchpad_finger_1_x < -0.3 -> robot.tseGrabber.prevState()
                gamepad2.touchpad_finger_1_y > 0.6 -> robot.tseGrabber.state = TseGrabber.TseGrabberState.STORAGE
            }
        }

        if ((gamepad2.y && !gamepad2CanControlExtras) || gamepad1CanControlAccessories && gamepad1.y) robot.fullIntakeSystem.resetDepositZero()

        // Control webcam servo
        robot.webcamServo.position = defaultWebcamPosition

        if (!gamepad1CanControlAccessories) {
            // Adjust drivetrain speed
            when {
                gamepad1Ex.wasJustPressed(DPAD_UP) -> if (speed < speedMax) speed++
                gamepad1Ex.wasJustPressed(DPAD_DOWN) -> if (speed > 1) speed--
                gamepad1Ex.wasJustPressed(DPAD_LEFT) -> cubicEnable = !cubicEnable
                toggleGamepad1TouchpadPress.lastState -> {
                    speed = when {
                        gamepad1.touchpad_finger_1_x < -0.5 -> 2
                        gamepad1.touchpad_finger_1_x < 0.0 -> 3
                        gamepad1.touchpad_finger_1_x < 0.5 -> 4
                        else -> 5
                    }
                }
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
        telemetry.addData("Time ?? (ms)", currentTime - lastTimeRead)
        telemetry.addLine()
        lastTimeRead = currentTime
        telemetry.addData("Drivetrain speed", "$speed out of $speedMax")
        telemetry.addData("Drivetrain speed adj", speed/speedMax)
        telemetry.addData("Drivetrain max rpm", maxRpm * (speed/speedMax))
        telemetry.addData("Cubic enable", cubicEnable)
        telemetry.addLine()
        telemetry.addData("Carousel speed (current)", robot.carouselMotorSystem.carouselMotor)
        telemetry.addData("Carousel speed", defaultCarouselSpeed)
        telemetry.addData("Carousel side (regular)", if (defaultCarouselSpeed < 0) "BLUE" else "RED")
        telemetry.addData("Carousel side (Duck Functions)", persistingAllianceColor)
        telemetry.addData("Carousel stop behavior", robot.carouselMotorSystem.carouselMotor.zeroPowerBehavior)
        telemetry.addData("TSE Grabber position", robot.tseGrabber.state)
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
        telemetry.addLine()
        telemetry.addData("Z Orientation", orientation.firstAngle)
        telemetry.addData("Y Orientation", orientation.secondAngle)
        telemetry.addData("X Orientation", orientation.thirdAngle)
        telemetry.addLine()
        telemetry.addData("Gamepad1 Touchpad Press", gamepad1.touchpad)
        telemetry.addData("Gamepad1 Finger1 X", gamepad1.touchpad_finger_1_x)
        telemetry.addData("Gamepad1 Finger1 Y", gamepad1.touchpad_finger_1_y)
        telemetry.addData("Gamepad1 Finger 1 Press", gamepad1.touchpad_finger_1)
        telemetry.addData("Gamepad1 Finger2 X", gamepad1.touchpad_finger_2_x)
        telemetry.addData("Gamepad1 Finger2 Y", gamepad1.touchpad_finger_2_y)
        telemetry.addData("Gamepad1 Finger 2 Press", gamepad1.touchpad_finger_2)
        telemetry.addData("Gamepad2 Touchpad Press", gamepad2.touchpad)
        telemetry.addData("Gamepad2 Finger1 X", gamepad2.touchpad_finger_1_x)
        telemetry.addData("Gamepad2 Finger1 Y", gamepad2.touchpad_finger_1_y)
        telemetry.addData("Gamepad1 Finger 1 Press", gamepad2.touchpad_finger_1)
        telemetry.addData("Gamepad2 Finger2 X", gamepad2.touchpad_finger_2_x)
        telemetry.addData("Gamepad2 Finger2 Y", gamepad2.touchpad_finger_2_y)
        telemetry.addData("Gamepad1 Finger 2 Press", gamepad2.touchpad_finger_2)

        robot.fullIntakeSystem.update()
    }
}