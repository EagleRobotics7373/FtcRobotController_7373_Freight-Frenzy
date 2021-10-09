package org.firstinspires.ftc.teamcode.basicstests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumDrive extends OpMode {

    DcMotor FRMotor;
    DcMotor FLMotor;
    DcMotor RRMotor;
    DcMotor RLMotor;
    DcMotor Arm;
    DcMotor Shooter;
    DcMotor Intake;
    Servo rightGrabber;
    Servo topGrabber;
    Servo Gate;
    double shooterOn;
    boolean aPressed;
    boolean yPressed;
    boolean dpadPressed;
    double shooterVelocity;
    double gateUp;






    public void moveDriveTrain(){

    }

    @Override
    public void init(){
        FRMotor = hardwareMap.get(DcMotor.class, "FrontRight");
        FLMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
        RRMotor = hardwareMap.get(DcMotor.class, "BackRight");
        RLMotor = hardwareMap.get(DcMotor.class, "BackLeft");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        rightGrabber = hardwareMap.get(Servo.class, "RightGrabber");
        topGrabber = hardwareMap.get(Servo.class,"TopGrabber");
        Gate = hardwareMap.get(Servo.class, "Gate");

        aPressed = false;
        yPressed = false;
        dpadPressed = false;
        shooterOn = 1;
        shooterVelocity = 1850;
        gateUp = 1;

        Gate.setPosition(1);
        Shooter.setPower(0);




    }

    @Override
    public void init_loop(){

    }
    @Override
    public void loop(){

        double vertical;
        double horizontal;
        double pivot;
        double speed;
        double armPower;

        //Speed Controls
        if (gamepad1.right_bumper){
            speed = 1;
        }
        else if(gamepad1.left_bumper){
            speed = 0.3;
        }
        else{
            speed = 0.5;
        }

        //Mecanum Drive
        vertical = gamepad1.left_stick_y * speed;
        horizontal = gamepad1.left_stick_x * speed;
        pivot = gamepad1.right_stick_x * speed;


        FRMotor.setPower(pivot - (vertical - horizontal));
        RRMotor.setPower(pivot - (vertical + horizontal));
        FLMotor.setPower(pivot + (vertical + horizontal));
        RLMotor.setPower(pivot + (vertical - horizontal));

        //Invert Controls



        //Arm
        armPower = (gamepad2.left_stick_y * 0.7);
        Arm.setPower(armPower);

        //Grabber
        if(gamepad2.left_bumper){
            topGrabber.setPosition(0);
            rightGrabber.setPosition(1);
        }
        if(gamepad2.right_bumper){
            topGrabber.setPosition(0.7);
            rightGrabber.setPosition(0.5);

        }

        //Shooter

        if (gamepad2.a){

            if(aPressed == false){

                shooterOn *= -1;
                aPressed = true;
            }
        }else {
            aPressed = false;
        }

        if (shooterOn == -1){
            Shooter.setPower(shooterVelocity);

        }else {
            Shooter.setPower(0);
        }

        //Shooter Velocity Controls
        if (gamepad2.dpad_up){
            if (dpadPressed == false){
                shooterVelocity += 50;
                dpadPressed = true;
            }
        } else if (gamepad2.dpad_down){
            if (dpadPressed == false){
                shooterVelocity -= 50;
                dpadPressed = true;
            }
        } else {
            dpadPressed = false;

        }


        //Gate


        if (gamepad2.y){

            if(yPressed == false){

                gateUp *= -1;
                yPressed = true;
            }
        }else {
            yPressed = false;
        }


        if (gateUp == -1){
            Gate.setPosition(0.4);
        } else {
            Gate.setPosition(1);

        }



        //Intake
        Intake.setPower(gamepad2.right_stick_y);




        //Telemetry
        telemetry.addData("speed", speed);
        telemetry.addData("Is Shooter On", shooterOn);
        telemetry.addData("Shooter Velocity", shooterVelocity);
        telemetry.addData("Dpad Pressed", dpadPressed);
        telemetry.addData("Gate Up", gateUp);
        telemetry.addData("yPressed", yPressed);
        telemetry.update();




    }
}

