package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.ToggleButtonWatcher
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BaseRobot

abstract class BaseAutonomous<T:BaseRobot>: LinearOpMode() {

    protected lateinit var robot           : T
    protected          val telem           : MultipleTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    protected lateinit var elapsedTime     : ElapsedTime

    protected val config = OpModeConfig(telemetry)

    protected fun operateMenu() {

        val dpadUpWatch = ToggleButtonWatcher {gamepad1.dpad_up}
        val dpadDownWatch = ToggleButtonWatcher {gamepad1.dpad_down}
        val dpadLeftWatch = ToggleButtonWatcher {gamepad1.dpad_left}
        val dpadRightWatch = ToggleButtonWatcher {gamepad1.dpad_right}

        config.update()

        while (!isStarted && !isStopRequested) {
            when {
                dpadUpWatch.invoke()    -> config.update(prevItem = true)
                dpadDownWatch.invoke()  -> config.update(nextItem = true)
                dpadLeftWatch.invoke()  -> config.update(iterBack = true)
                dpadRightWatch.invoke() -> config.update(iterFw   = true)
            }

        }
    }

    /**
     * Robot strafes for desired distance until motors have reached target.
     * Forwards distances and power to [robot.holonomic]
     */
    protected fun drive(x: Double, y: Double, power: Double) {
        robot.holonomic.runUsingEncoder(x, y, power)
        val originalRuntime = runtime
        while (opModeIsActive() && robot.holonomic.motorsAreBusy() && runtime - originalRuntime < 3) robot.holonomicRR?.update();
        robot.holonomic.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    /**
     * Robot strafes with specified power until desired time length is reached.
     * Forwards power to [robot.holonomic]
     */
    protected fun timeDrive(x: Double, y: Double, z: Double, timeMs: Long) {
        val startingRuntime = System.currentTimeMillis()
        robot.holonomic.runWithoutEncoder(x, y, z)
        while (System.currentTimeMillis() - startingRuntime < timeMs) robot.holonomicRR?.update()
        robot.holonomic.stop()
        robot.holonomic.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER)

    }

    protected fun forDurationMs(duration: Long, action: ()->Unit) {
        val end = System.currentTimeMillis() + duration
        while (System.currentTimeMillis() < end) action.invoke()
    }

    protected fun builder() = robot.holonomicRR!!.trajectoryBuilder()
    protected fun builder(tangent: Double) = robot.holonomicRR!!.trajectoryBuilder(tangent)

    protected fun BaseTrajectoryBuilder<TrajectoryBuilder>.buildAndRun(vararg waypointActions: Pair<Double, ()->Unit>) =
            robot.holonomicRR!!.followTrajectorySync(this.build(), waypointActions.toList())
}