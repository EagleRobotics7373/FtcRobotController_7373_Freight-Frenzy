package org.firstinspires.ftc.teamcode.basicstests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class SensorTest extends OpMode{

    ColorSensor colorSensor;

    @Override
    public void init(){
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
    }

    @Override
    public void loop() {
        int red = colorSensor.red();

        telemetry.addData("Red", red);
    }

}