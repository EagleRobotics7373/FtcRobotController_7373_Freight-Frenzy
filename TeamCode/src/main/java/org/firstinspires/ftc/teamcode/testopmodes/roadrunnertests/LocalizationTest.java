package org.firstinspires.ftc.teamcode.testopmodes.roadrunnertests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.robot.robotcore.BaseRobot;
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.HolonomicRR;

import static org.firstinspires.ftc.teamcode.library.robot.robotcore.RobotProvider.providePresetRobot;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "rr_cfg")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        BaseRobot robot = providePresetRobot(hardwareMap);
        HolonomicRR drive = robot.getHolonomicRR();

        boolean fast = false;
        boolean reverse = true;
        boolean fod = false;

        double baselineAngle = 0.0;



        waitForStart();

        while (!isStopRequested()) {
            robot.holonomic.runWithoutEncoderVectored(gamepad1.left_stick_x * (fast?1.0:0.4) * (reverse?-1.0:1.0), -gamepad1.left_stick_y * (fast?1.0:0.4) * (reverse?-1.0:1.0), gamepad1.right_stick_x * (fast?1.0:0.4), (fod?(baselineAngle - drive.getExternalHeading()):0.0));
            drive.update();

            if (gamepad1.dpad_up) fast = true;
            else if (gamepad1.dpad_down) fast = false;
//            Pose2d poseEstimate = drive.getPoseEstimate();
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            if (gamepad1.a)      { reverse = false; fod = false; }
            else if (gamepad1.b) { reverse = true; fod = false; }
            else if (gamepad1.x) { fod = true; }

            if (gamepad1.y) baselineAngle = drive.getExternalHeading();
        }
    }
}
