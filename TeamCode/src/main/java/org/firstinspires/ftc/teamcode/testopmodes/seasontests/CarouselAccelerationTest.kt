package org.firstinspires.ftc.teamcode.testopmodes.seasontests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor
import org.firstinspires.ftc.teamcode.library.functions.DashboardVar
import kotlin.math.exp

@TeleOp
class CarouselAccelerationTest: OpMode() {
    lateinit var carouselMotor: DcMotorEx

    private var motionStart: Double = Double.NEGATIVE_INFINITY
    private var accelStart: Double by DashboardVar(1.0, "accelStart", this::class) { it >= 0.0 }
    private var accelDuration1: Double by DashboardVar(1.0, "accelDuration1", this::class) { it > 0.0 }
    private var accelDelayDuration: Double by DashboardVar(0.0, "accelDelayDuration", this::class) { it >= 0.0 }
    private var accelDuration2: Double by DashboardVar(1.0, "accelDuration2", this::class) { it > 0.0 }
    private var constDuration: Double by DashboardVar(1.0, "constDuration", this::class) { it > 0.0 }

    private var initialSpeed: Double by DashboardVar(0.5, "initialSpeed", this::class) { it > 0.0 }
    private var intermediateSpeed: Double by DashboardVar(0.9, "intermediateSpeed", this::class) { it > 0.0 }
    private var finalSpeed: Double by DashboardVar(1.0, "finalSpeed", this::class) { it > 0.0 }

    private var allianceColor: AllianceColor by DashboardVar(AllianceColor.BLUE, "allianceColor", this::class)

    private val k1: Double
        get() = -12.0/accelDuration1

    private val k2: Double
        get() = -12.0/accelDuration2

    private val deltaVel1: Double
        get() = intermediateSpeed - initialSpeed

    private val deltaVel2: Double
        get() = finalSpeed - intermediateSpeed

    private val a: Double
        get() = accelStart + accelDuration1/2

    private val b: Double
        get() = accelStart + accelDuration1 + accelDelayDuration + accelDuration2/2

    override fun init() {
        carouselMotor = hardwareMap.get(DcMotorEx::class.java, "carouselMotor")

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

//        carouselMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
//        carouselMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDFCoefficients(p, i, d, f))
    }

    override fun loop() {
        when {
            gamepad1.a -> motionStart = System.currentTimeMillis() / 1000.0
            gamepad1.y -> motionStart = Double.NEGATIVE_INFINITY
            gamepad1.b -> allianceColor = AllianceColor.RED
            gamepad1.x -> allianceColor = AllianceColor.BLUE
        }

        val elapsedTime = (System.currentTimeMillis() / 1000.0) - motionStart
        carouselMotor.power = when (elapsedTime) {
            in 0.0..accelStart -> initialSpeed
            in accelStart..(accelStart + accelDuration1) -> initialSpeed + (deltaVel1 / (1 + exp(k1 * (elapsedTime - a))))
            in (accelStart + accelDuration1)..(accelStart + accelDuration1 + accelDelayDuration) -> intermediateSpeed
            in (accelStart + accelDuration1 + accelDelayDuration)..(accelStart + accelDuration1 + accelDelayDuration + accelDuration2) -> intermediateSpeed + (deltaVel2 / (1 + exp(k2 * (elapsedTime - b))))
            in (accelStart + accelDuration1 + accelDelayDuration)..(accelStart + accelDuration1 + accelDelayDuration + constDuration) -> finalSpeed
            else -> 0.0
        } * when(allianceColor) {
            AllianceColor.RED -> 1.0
            AllianceColor.BLUE -> -1.0
        }

        telemetry.addData("carousel speed", carouselMotor.power)
        telemetry.addData("elapsed time", elapsedTime)
        telemetry.addData("total Duck time", accelDuration1 + accelDelayDuration + accelDuration2 + constDuration)
    }

}