package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem

class ExtThinBot(_hardwareMap: HardwareMap): BaseRobot(_hardwareMap) {

    @JvmField val intakeMotor1 : DcMotorEx = hwInit("intakeMotor1")
    @JvmField val intakeMotor2 : DcMotorEx = hwInit("intakeMotor2")
    @JvmField val carouselMotor : DcMotorEx = hwInit("carouselMotor")
    @JvmField val depositLiftMotor : DcMotorEx = hwInit("depositLiftMotor")
    @JvmField val depositServo : Servo = hwInit("depositServo")

    @JvmField val fullIntakeSystem = FullIntakeSystem(depositLiftMotor, intakeMotor1, intakeMotor2)

}