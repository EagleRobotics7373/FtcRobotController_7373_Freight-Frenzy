package org.firstinspires.ftc.teamcode.library.robot.systems.lt

import com.qualcomm.robotcore.hardware.Servo

class OdometryLiftSystem(
        private val odometryServoLeft: Servo,
        private val odometryServoCenter: Servo
) {

    private val raisePosCenter: Double = 0.37
    private val lowerPosCenter: Double = 1.0
    private val raisePosLeft: Double = 0.1
    private val lowerPosLeft: Double = 0.5

    fun raise() {
        odometryServoCenter.position = raisePosCenter
        odometryServoLeft.position = raisePosLeft
    }

    fun lower() {
        odometryServoCenter.position = lowerPosCenter
        odometryServoLeft.position = lowerPosLeft
    }
}