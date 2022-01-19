package org.firstinspires.ftc.teamcode.library.robot.systems.lt

import com.qualcomm.robotcore.hardware.Servo

class OdometryLiftSystem(
        private val odometryServoLeft: Servo,
        private val odometryServoCenter: Servo
) {

    private val raisePos: Double = 0.0
    private val lowerPos: Double = 0.0

    fun raise() {
        odometryServoCenter.position = raisePos
        odometryServoLeft.position = raisePos
    }

    fun lower() {
        odometryServoCenter.position = lowerPos
        odometryServoLeft.position = lowerPos
    }
}