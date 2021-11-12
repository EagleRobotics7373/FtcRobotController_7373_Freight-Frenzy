package org.firstinspires.ftc.teamcode.library.robot.systems.meet2

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.functions.DashboardVar
import kotlin.math.absoluteValue

class FullIntakeSystem(
        private val depositLiftMotor: DcMotorEx,
        private val depositServo: Servo,
        private val intakeMotor1: DcMotorEx,
        private val intakeMotor2: DcMotorEx
) {
    // The height at which we can still consider the deposit box lowered, for intaking
    private var depositLiftIsLoweredCutoffTicks =-100/*by DashboardVar(100, "depositLiftIsLoweredCutoffTicks", this::class)*/

    // Deposit servo positions
    private var depositServoIn = 0.53/*by DashboardVar(0.58, "depositServoIn", this::class)*/
    private var depositServoMid = 0.4/*by DashboardVar(0.4, "depositServoMid", this::class)*/
    private var depositServoOut = 0.1/*by DashboardVar(0.2, "depositServoOut", this::class)*/

    var depositServoIsExtended: Boolean = false

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
    fun depositLiftAuto(height: DepositLiftPosition, power: Double) {
        depositLiftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        depositLiftMotor.power = power
        depositLiftMotor.targetPosition = height.ticks
    }

    // Refresh deposit lift motor state
    fun update() {
        depositServo.position =
                if (depositServoIsExtended) depositServoOut
                else if (depositLiftIsLowered) depositServoIn
                else depositServoMid
    }

    // Whether intaking should be blocked if the deposit box is not lowered
    val shouldCheckDepositHeightUponIntake = true

    // Whether reverse-driving the spool should be blocked
    val shouldBlockReverseDrivingSpool = true

    // Check whether the deposit lift is actually lowered
    val depositLiftIsLowered: Boolean
    get() = depositLiftMotor.currentPosition.absoluteValue < depositLiftIsLoweredCutoffTicks.absoluteValue

    fun resetDepositZero() {
        depositLiftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    enum class DepositLiftPosition(val ticks: Int) {
        LOW(0),
        MIDDLE(-400),
        HIGH(-900),
        TSE(0);
    }
}