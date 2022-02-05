package org.firstinspires.ftc.teamcode.library.robot.systems.st

import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.Companion.persistingAllianceColor
import kotlin.math.exp

class CarouselMotorSystem
constructor(val carouselMotor : DcMotorEx)
{
    var motionStart: Double = Double.NEGATIVE_INFINITY
    private var accelStart: Double = 0.01
    private var accelDuration1: Double = 0.25
    private var accelDelayDuration: Double = 0.0
    private var accelDuration2: Double = 0.75
    private var constDuration: Double = 1.0

    private var initialSpeed: Double = 0.5
    private var intermediateSpeed: Double = 0.9
    private var finalSpeed: Double = 1.0

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