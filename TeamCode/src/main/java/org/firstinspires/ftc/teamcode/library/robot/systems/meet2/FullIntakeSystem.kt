package org.firstinspires.ftc.teamcode.library.robot.systems.meet2

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

class FullIntakeSystem(
        private val depositLiftMotor: DcMotorEx,
        private val intakeMotor1: DcMotorEx,
        private val intakeMotor2: DcMotorEx
) {


    // Run the intake motors at the desired power
    fun intakeManual(motor1Power: Double, motor2Power: Double) {
        if (shouldCheckDepositHeightUponIntake && !depositLiftIsLowered) return
        intakeMotor1.power = motor1Power
        intakeMotor2.power = motor2Power
    }

    // Run the intake motors at the same desired power
    fun intakeManual(bothMotorsPower: Double) {
        intakeManual(bothMotorsPower, bothMotorsPower)
    }

    // Run the deposit lift motor at the desired power
    fun depositLiftManual(power: Double) {
//        if (depositLiftMotor.currentPosition < 0 && power < 0) return
        depositLiftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        depositLiftMotor.power = power
    }

    // Automatically raise/lower the deposit lift motor to the desired position
    fun depositLiftAuto(height: DepositPosition, power: Double) {
        depositLiftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        depositLiftMotor.power = power
        depositLiftMotor.targetPosition = height.ticks
    }

    // Refresh deposit lift motor state
    fun update() {
        if (depositLiftMotor.mode == DcMotor.RunMode.RUN_TO_POSITION && !depositLiftMotor.isBusy) {
            depositLiftManual(0.0)
        }
    }

    // Whether intaking should be blocked if the deposit box is not lowered
    val shouldCheckDepositHeightUponIntake = true

    // Whether reverse-driving the spool should be blocked
    val shouldBlockReverseDrivingSpool = true

    // The height at which we can still consider the deposit box lowered, for intaking
    val depositLiftIsLoweredCutoffTicks = 100

    // Check whether the deposit lift is actually lowered
    val depositLiftIsLowered: Boolean
    get() = depositLiftMotor.currentPosition < depositLiftIsLoweredCutoffTicks

    fun resetDepositZero() {
        depositLiftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    enum class DepositPosition(val ticks: Int) {
        LOW(0),
        MIDDLE(10),
        HIGH(20),
        TSE(30);
    }
}