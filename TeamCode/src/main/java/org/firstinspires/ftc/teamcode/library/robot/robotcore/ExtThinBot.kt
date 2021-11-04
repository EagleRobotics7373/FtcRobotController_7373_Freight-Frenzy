package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class ExtThinBot(_hardwareMap: HardwareMap): BaseRobot(_hardwareMap) {

    @JvmField val intakeMotor : DcMotorEx = hwInit("intakeMotor")
    @JvmField val carouselMotor : DcMotorEx = hwInit("carouselMotor")
    @JvmField val linearActuatorMotor : DcMotorEx = hwInit("linearActuatorMotor")
    @JvmField val linearActuatorServo : Servo = hwInit("linearActuatorServo")
    @JvmField val outServo : Servo = hwInit("outServo")

}