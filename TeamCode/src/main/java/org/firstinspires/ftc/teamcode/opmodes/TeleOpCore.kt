package org.firstinspires.ftc.teamcode.opmodes

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.DashboardVar
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot
import kotlin.math.absoluteValue

@TeleOp(name="TeleOpCore (Kotlin)")
class TeleOpCore: OpMode() {

    lateinit var robot: ExtThinBot
    val gamepad1Ex = GamepadEx(gamepad1)
    val gamepad2Ex = GamepadEx(gamepad2)

    private var reverse by DashboardVar(false, "reverse", this::class)
    private var speed by DashboardVar(1, "speed", this::class) {it in 1..3}

    private var carouselRPS by DashboardVar(10, "carouselRPS", this::class)
    private var carouselTPS by DashboardVar(carouselRPS * 145.1, "carouselTPS", this::class)
    private var defaultCarouselSpeed by DashboardVar(-0.25, "defaultCarouselSpeed", this::class)

    private var depositServoIn by DashboardVar(0.6, "depositServoIn", this::class)
    private var depositServoOut by DashboardVar(0.32, "depositServoOut", this::class)

    override fun init() {
        robot = ExtThinBot(hardwareMap)
    }

    override fun loop() {

        gamepad1Ex.readButtons()
        gamepad2Ex.readButtons()

        // Control deposit servo
        when {
            gamepad2.a -> robot.depositServo.position = 0.0
            gamepad2.b -> robot.depositServo.position = 0.0
        }

        // Control both intake motors
        val dualIntakeMotorPower = when {
            gamepad2.left_trigger > 0.05 && !gamepad2.right_bumper -> -gamepad2.left_trigger.toDouble()
            gamepad2.right_trigger > 0.05 -> gamepad2.right_trigger.toDouble()
            gamepad1.left_trigger > 0.05 -> gamepad1.left_trigger.toDouble()
            gamepad1.right_trigger > 0.05 -> gamepad1.right_trigger.toDouble()
            else -> 0.0
        }
        if (dualIntakeMotorPower > 0)
            robot.fullIntakeSystem.intakeManual(dualIntakeMotorPower)
        else if (!gamepad2.right_bumper)
            robot.fullIntakeSystem.intakeManual(gamepad2.left_stick_y.toDouble(), gamepad2.right_stick_y.toDouble())
        else
            robot.fullIntakeSystem.intakeManual(0.0)

        // Control carousel motor
        robot.carouselMotor.power = when {
            gamepad2.right_bumper && gamepad2.left_stick_y.absoluteValue > 0.05 -> gamepad2.left_stick_y.toDouble()
            gamepad2.left_bumper -> defaultCarouselSpeed
            else -> 0.0
        }

        // Adjust default carousel speed
        if (gamepad2.right_bumper) {
            when {
                gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP) -> defaultCarouselSpeed += 0.05
                gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) -> defaultCarouselSpeed -= 0.05
                gamepad2Ex.wasJustPressed(GamepadKeys.Button.X) -> defaultCarouselSpeed *= -1.0
            }
        }

        // Control deposit lift
        val depositLiftPower = if (gamepad2.right_bumper) gamepad2.right_stick_y.toDouble() else 0.0
        robot.fullIntakeSystem.depositLiftManual(depositLiftPower)
        if (gamepad2.y) robot.fullIntakeSystem.resetDepositZero()

        // Adjust drivetrain speed
        when {
            gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP) -> if (speed < 3) speed++
            gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) -> if (speed > 1) speed--
        }

        // Reverse drivetrain
        when {
            gamepad1.a -> reverse = false
            gamepad1.b -> reverse = true
            gamepad1Ex.wasJustPressed(GamepadKeys.Button.X) -> reverse = !reverse
        }

        val vertical = -gamepad1.left_stick_y.toDouble() * speed/3 * if (reverse) 1 else -1
        val horizontal = gamepad1.left_stick_x.toDouble() * speed/3 * if (reverse) 1 else -1
        val pivot = gamepad1.right_stick_x.toDouble() * speed/3


//        robot.holonomic.runWithoutEncoderVectored(horizontal, vertical, pivot, 0);
        robot.frontRightMotor.power = pivot - vertical + horizontal
        robot.backRightMotor.power = pivot - vertical - horizontal
        robot.frontLeftMotor.power = pivot + vertical + horizontal
        robot.backLeftMotor.power = pivot + vertical - horizontal

        telemetry.addData("Carousel speed", defaultCarouselSpeed)
        telemetry.addData("Carousel side", if (defaultCarouselSpeed < 0) "BLUE" else "RED")
        telemetry.addLine()
        telemetry.addData("Deposit lift power", depositLiftPower)
        telemetry.addData("Deposit lift position", robot.depositLiftMotor.currentPosition)
        telemetry.addData("Combined intake motor power", dualIntakeMotorPower)
    }
}