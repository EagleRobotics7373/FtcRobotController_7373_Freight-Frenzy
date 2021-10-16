package org.firstinspires.ftc.teamcode.testopmodes.drivetests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ThinBotTest extends OpMode {
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;

    @Override
    public void init(){
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    }

    @Override
    public void loop(){
        double vertical;
        double horizontal;
        double pivot;
        double speed;

        //Speed Controls
        if (gamepad1.right_bumper)      speed = 1;
        else if (gamepad1.left_bumper)  speed = 0.3;
        else                            speed = 0.5;

        //Mecanum Drive
        vertical = gamepad1.left_stick_y * speed;
        horizontal = gamepad1.left_stick_x * speed;
        pivot = gamepad1.right_stick_x * speed;


        frontRightMotor.setPower(pivot - vertical - horizontal);
        backRightMotor.setPower(pivot - vertical + horizontal);
        frontLeftMotor.setPower(pivot + vertical + horizontal);
        backLeftMotor.setPower(pivot + vertical - horizontal);

    }
}
