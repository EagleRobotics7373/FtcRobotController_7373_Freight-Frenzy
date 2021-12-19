package org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.robot.robotcore.RobotProvider.providePresetRobot
import org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests.RoadRunnerTestConstants.STRAIGHT_TEST_DIST

@TeleOp(group="rr_cfg")
class StraightTest : LinearOpMode() {
    override fun runOpMode() {
        val robot = providePresetRobot(hardwareMap)
        val drive = robot.holonomicRR!!
        waitForStart()

        val trajectory =
                drive.trajectoryBuilder()
                        .forward(STRAIGHT_TEST_DIST)
                        .build()

        if (isStopRequested) return

        drive.followTrajectorySync(trajectory)
        while (!isStopRequested) drive.update()
    }

}