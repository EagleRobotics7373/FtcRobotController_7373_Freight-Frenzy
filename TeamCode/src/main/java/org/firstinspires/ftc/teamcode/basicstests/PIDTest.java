package org.firstinspires.ftc.teamcode.basicstests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PIDTest extends OpMode {
    DcMotorEx wheel;

    static double targetVelocity = 1200;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0.2, 0, 0);
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    @Override
    public void init() {
        wheel = hardwareMap.get(DcMotorEx.class, "Wheel");

        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop(){

        PID(targetVelocity);

        telemetry.addData("currentVelocity", wheel.getVelocity());
        telemetry.addData("targetVelocity", targetVelocity);

        telemetry.update();

    }
    double integral = 0;
    double lastError = 0;
    public void PID(double targetVelocity) {

        PIDTimer.reset();
        double currentVelocity = wheel.getVelocity();

        double error = targetVelocity - currentVelocity;

        integral += error * PIDTimer.time();
        
        double deltaError = error - lastError;
        
        //FINDS RATE OF CHANGE
        double derivative = deltaError/PIDTimer.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integral;
        pidGains.d = pidCoeffs.d = derivative;

        wheel.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);

        lastError = error;
        



    }
}


