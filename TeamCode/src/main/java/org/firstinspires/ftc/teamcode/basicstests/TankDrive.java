package org.firstinspires.ftc.teamcode.basicstests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="strafe test WOW")
public class TankDrive extends OpMode {

    DcMotor FRMotor;
    DcMotor FLMotor;
    DcMotor RRMotor;
    DcMotor RLMotor;
    boolean strafeOn = true;
    double horizontal;

    public void moveDriveTrain() {

    }

    @Override
    public void init() {
        FRMotor = hardwareMap.get(DcMotor.class, "FrontRight");
        FLMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
        RRMotor = hardwareMap.get(DcMotor.class, "BackRight");
        RLMotor = hardwareMap.get(DcMotor.class, "BackLeft");


    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {

        double vertical;
        double pivot;
        double speed;

        //Speed Controls
        if (gamepad1.right_bumper) {
            speed = 1;
        } else if (gamepad1.left_bumper) {
            speed = 0.3;
        } else {
            speed = 0.5;
        }
        //Strafing
        if (gamepad1.a) {
            strafeOn = true;
        } else if (gamepad1.b) {
            strafeOn = false;
        }

        if (strafeOn) horizontal = gamepad1.left_stick_x;
        else horizontal = 0;


        //Mecanum Drive
        vertical = gamepad1.left_stick_y * speed;
        pivot = -gamepad1.right_stick_x * speed;
        horizontal = horizontal * speed;


        FRMotor.setPower(pivot - vertical - horizontal);
        RRMotor.setPower(pivot - vertical + horizontal);
        FLMotor.setPower(pivot + vertical + horizontal);
        RLMotor.setPower(pivot + vertical - horizontal);


        //Telemetry
        telemetry.addData("speed", speed);
        telemetry.addData("Is strafe on", strafeOn);
        telemetry.update();


    }
}