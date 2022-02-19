package org.firstinspires.ftc.teamcode.library.robot.systems.st

import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.Companion.persistingAllianceColor
import org.firstinspires.ftc.teamcode.library.functions.DashboardVar
import kotlin.math.exp

class CarouselMotorSystem
constructor(val carouselMotor : DcMotorEx)
{
    var motionStart: Double = Double.NEGATIVE_INFINITY
    private var accelStart: Double by DashboardVar(0.5, "accelStart", this::class)
    private var accelDuration1: Double by DashboardVar(0.20, "accelDuration1", this::class)
    private var accelDelayDuration: Double by DashboardVar(0.4, "accelDelayDuration", this::class)
    private var accelDuration2: Double by DashboardVar(0.20, "accelDuration2", this::class)
    private var constDuration: Double by DashboardVar(0.5, "constDuration", this::class)

    private var initialSpeed: Double by DashboardVar(0.6, "initialSpeed", this::class)
    private var intermediateSpeed: Double by DashboardVar(0.8, "intermediateSpeed", this::class)
    private var finalSpeed: Double by DashboardVar(0.9, "finalSpeed", this::class)

    private var velocityConstant: Double = 2796.04

    private val k1: Double get() = -12.0/accelDuration1
    private val k2: Double get() = -12.0/accelDuration2
    private val deltaVel1: Double get() = intermediateSpeed - initialSpeed
    private val deltaVel2: Double get() = finalSpeed - intermediateSpeed
    private val a: Double get() = accelStart + accelDuration1/2
    private val b: Double get() = accelStart + accelDuration1 + accelDelayDuration + accelDuration2/2

    fun run() {
        val elapsedTime = (System.currentTimeMillis() / 1000.0) - motionStart
        carouselMotor.velocity = when (elapsedTime) {
            in 0.0..accelStart -> initialSpeed
            in accelStart..(accelStart + accelDuration1) -> initialSpeed + (deltaVel1 / (1 + exp(k1 * (elapsedTime - a))))
            in (accelStart + accelDuration1)..(accelStart + accelDuration1 + accelDelayDuration) -> intermediateSpeed
            in (accelStart + accelDuration1 + accelDelayDuration)..(accelStart + accelDuration1 + accelDelayDuration + accelDuration2) -> intermediateSpeed + (deltaVel2 / (1 + exp(k2 * (elapsedTime - b))))
            in (accelStart + accelDuration1 + accelDelayDuration)..(accelStart + accelDuration1 + accelDelayDuration + constDuration) -> finalSpeed
            else -> 0.0
        } * when(persistingAllianceColor) {
            AllianceColor.RED -> 1.0
            AllianceColor.BLUE -> -1.0
        } * velocityConstant

    }
    fun start() {
        motionStart = System.currentTimeMillis() / 1000.0
    }

}