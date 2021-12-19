package org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.HolonomicRR;

import static org.firstinspires.ftc.teamcode.library.robot.robotcore.RobotProvider.providePresetRobot;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@TeleOp(group = "rr_cfg")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        HolonomicRR drive = providePresetRobot(hardwareMap).getHolonomicRR();

        waitForStart();

        if (isStopRequested()) return;

        drive.turnSync(Math.toRadians(ANGLE));

        while (!isStopRequested()) drive.update();
    }
}
