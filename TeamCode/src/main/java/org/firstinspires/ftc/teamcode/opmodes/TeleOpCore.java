package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kotlin.extensions.gamepad.GamepadExExtKt;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.library.functions.ToggleButtonWatcher;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot;
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.HolonomicImpl;

import kotlin.jvm.functions.Function0;

@Config
@TeleOp
public class TeleOpCore extends OpMode {

    private ExtThinBot robot;


    public static double IN = 0.6;
    public static double OUT = 0.32;

    public static double LIN_POS_IN = 1.0;
    public static double LIN_POS_OUT = 0.0;

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static double carouselRPS = 10;
    double carouselTPS = carouselRPS * 145.1;

    public static boolean reverse = false;

    public static double defaultCarouselSpeed = -0.25;

    private final GamepadEx gamepad1Ex = new GamepadEx(gamepad1);
    private final GamepadEx gamepad2Ex = new GamepadEx(gamepad1);

    @Override
    public void init(){
        robot = new ExtThinBot(hardwareMap);
    }


    @Override
    public void loop(){
        if(gamepad2.a) robot.outServo.setPosition(IN);
        else if(gamepad2.b) robot.outServo.setPosition(OUT);

        if(gamepad2.left_trigger > 0.05) robot.intakeMotor.setPower(-gamepad2.left_trigger);
        else if(gamepad2.right_trigger > 0.05) robot.intakeMotor.setPower(gamepad2.right_trigger);
        if(gamepad1.left_trigger > 0.05) robot.intakeMotor.setPower(-gamepad1.left_trigger);
        else if(gamepad1.right_trigger > 0.05) robot.intakeMotor.setPower(gamepad1.right_trigger);
        else robot.intakeMotor.setPower(0.0);

        if (Math.abs(gamepad2.left_stick_y) > 0.05) robot.carouselMotor.setPower(gamepad2.left_stick_y);
        else if (gamepad2.left_bumper) robot.carouselMotor.setPower(defaultCarouselSpeed);
        else robot.carouselMotor.setPower(0);

        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP)) defaultCarouselSpeed += 0.05;
        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) defaultCarouselSpeed -= 0.05;
        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.X)) defaultCarouselSpeed *= -1;
        telemetry.addData("Carousel speed", defaultCarouselSpeed);
        telemetry.addData("Carousel side", defaultCarouselSpeed < 0 ? "BLUE":"RED");


        robot.linearActuatorMotor.setPower(gamepad2.right_stick_y * 0.20);

        if (gamepad2.x) robot.linearActuatorServo.setPosition(LIN_POS_OUT);
        if (gamepad2.y) robot.linearActuatorServo.setPosition(LIN_POS_IN);

        double vertical;
        double horizontal;
        double pivot;        double speed;

        //Speed Controls
        if (gamepad1.right_bumper)      speed = 1;
        else if (gamepad1.left_bumper)  speed = 0.3;
        else                            speed = 0.5;

        if (gamepad1.a) reverse = true;
        else if (gamepad1.b) reverse = false;

        //Mecanum Drive
        vertical = -gamepad1.left_stick_y * speed * (reverse ? 1: -1);
        horizontal = gamepad1.left_stick_x * speed * (reverse ? 1: -1);
        pivot = gamepad1.right_stick_x * speed;

//        robot.holonomic.runWithoutEncoderVectored(horizontal, vertical, pivot, 0);
        robot.frontRightMotor.setPower(pivot - vertical + horizontal);
        robot.backRightMotor.setPower(pivot - vertical - horizontal);
        robot.frontLeftMotor.setPower(pivot + vertical + horizontal);
        robot.backLeftMotor.setPower(pivot + vertical - horizontal);

        gamepad1Ex.readButtons();
        gamepad2Ex.readButtons();
    }
}
