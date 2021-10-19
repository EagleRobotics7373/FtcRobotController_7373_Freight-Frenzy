package org.firstinspires.ftc.teamcode.testopmodes.seasontests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class CarouselTest extends OpMode {
    DcMotor carouselMotor;

    boolean usePredeterminedSpeed = false;
    double predeterminedSpeed = 0.5;

    @Override
    public void init() {
        carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");


    }

    @Override
    public void loop() {
        double triggerInput = gamepad1.left_trigger;
        telemetry.addData("input",triggerInput);

        double speedSentToMotor = usePredeterminedSpeed ? predeterminedSpeed : triggerInput;
        telemetry.addData("predetermined", predeterminedSpeed);
        telemetry.addData("sent to motor", speedSentToMotor);
        telemetry.addData("use predetermined", usePredeterminedSpeed);

        if (gamepad1.a) usePredeterminedSpeed = true;
        else if (gamepad1.b) usePredeterminedSpeed = false;
        else if (gamepad1.x) predeterminedSpeed = triggerInput;

        carouselMotor.setPower(speedSentToMotor);
    }


}