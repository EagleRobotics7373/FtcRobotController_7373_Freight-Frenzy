package org.firstinspires.ftc.teamcode.testopmodes.seasontests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp
public class CarouselTest extends OpMode {
    DcMotorEx carouselMotor;

    boolean usePredeterminedSpeed = false;
    public static double predeterminedSpeed = 10;
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;
    public double predeterminedSpeedInTick = predeterminedSpeed * 145.1;

    @Override
    public void init() {
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouselMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
    }

    @Override
    public void loop() {
        double triggerInput = gamepad1.left_trigger;
        telemetry.addData("input",triggerInput);

        double speedSentToMotor = usePredeterminedSpeed ? predeterminedSpeedInTick : triggerInput;
        telemetry.addData("predetermined (RPS)", predeterminedSpeed);
        telemetry.addData("sent to motor", speedSentToMotor);
        telemetry.addData("use predetermined", usePredeterminedSpeed);

        if (gamepad1.a) usePredeterminedSpeed = true;
        else if (gamepad1.b) usePredeterminedSpeed = false;
        else if (gamepad1.x) predeterminedSpeed = triggerInput;

        carouselMotor.setVelocity(speedSentToMotor);
    }


}