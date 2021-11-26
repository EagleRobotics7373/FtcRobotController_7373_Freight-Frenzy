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

    boolean modeIndividualIsOn = false;
    int selectedMotor = 0;
    String[] motorNames;
    DcMotor[] motors;

    @Override
    public void init(){
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");

        motorNames = new String[]{"frontRightMotor", "frontLeftMotor", "backRightMotor", "backLeftMotor"};
        motors = new DcMotor[]{frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor};
    }

    @Override
    public void loop(){
        if (modeIndividualIsOn) {

            telemetry.addData("Mode", "Individual Control");
            telemetry.addLine();
            telemetry.addData("Motor", motorNames[selectedMotor]);
            telemetry.addData("Power", motors[selectedMotor].getPower());
            telemetry.addData("Current Position", motors[selectedMotor].getCurrentPosition());

            float power = gamepad1.left_stick_y;
            for (int i = 0; i < motors.length; i++) {
                // Set power of motor to 0 if it is not the selected motor
                motors[i].setPower(i == selectedMotor ? power : 0);

                // Pass motor name and current encoder position to telemetry
                telemetry.addData(motorNames[i], motors[i].getCurrentPosition());
            }

            if (gamepad1.dpad_up) selectedMotor = 0;
            else if (gamepad1.dpad_right) selectedMotor = 1;
            else if (gamepad1.dpad_down) selectedMotor = 2;
            else if (gamepad1.dpad_left) selectedMotor = 3;

        } else {
            telemetry.addData("Mode", "Drivetrain Control");
            double vertical;
            double horizontal;
            double pivot;
            double speed;

            //Speed Controls
            if (gamepad1.right_bumper)      speed = 1;
            else if (gamepad1.left_bumper)  speed = 0.3;
            else                            speed = 0.5;

            //Mecanum Drive
            vertical = -gamepad1.left_stick_y * speed;
            horizontal = gamepad1.left_stick_x * speed;
            pivot = gamepad1.right_stick_x * speed;


            frontRightMotor.setPower(pivot - vertical + horizontal);
            backRightMotor.setPower(pivot - vertical - horizontal);
            frontLeftMotor.setPower(pivot + vertical + horizontal);
            backLeftMotor.setPower(pivot + vertical - horizontal);
        }

        if (gamepad1.a) modeIndividualIsOn = false;
        else if (gamepad1.b) modeIndividualIsOn = true;
    }
}
