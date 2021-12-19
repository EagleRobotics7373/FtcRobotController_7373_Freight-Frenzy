package org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.HolonomicRR;

import static org.firstinspires.ftc.teamcode.library.robot.robotcore.RobotProvider.providePresetRobot;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "rr_cfg")
@Config
public class SidewaysTest extends LinearOpMode {
    public static double distY = 10;
    @Override
    public void runOpMode() throws InterruptedException {
        HolonomicRR drive = providePresetRobot(hardwareMap).getHolonomicRR();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(distY)
                        .build()
        );

//        sleep(2000);
//
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .reverse()
//                        .splineTo(new Pose2d(0, 0, 0))
//                        .build()
//        );

        while (!isStopRequested()) drive.update();
    }
}
