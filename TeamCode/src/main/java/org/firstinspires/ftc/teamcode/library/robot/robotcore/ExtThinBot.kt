package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.HolonomicRR
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.library.robot.systems.lt.OdometryLiftSystem
import org.firstinspires.ftc.teamcode.library.robot.systems.lt.TseGrabber
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem

class ExtThinBot(_hardwareMap: HardwareMap): BaseRobot(_hardwareMap) {

    @JvmField val intakeMotor1 : DcMotorEx = hwInit("intakeMotor1")
    @JvmField val intakeMotor2 : DcMotorEx = hwInit("intakeMotor2")
    @JvmField val carouselMotor : DcMotorEx = hwInit("carouselMotor")
    @JvmField val depositLiftMotor : DcMotorEx = hwInit("depositLiftMotor")
    @JvmField val depositServo : Servo = hwInit("depositServo")
    @JvmField val webcamServo : Servo = hwInit("webcamServo")
    @JvmField val odometryLift: OdometryLiftSystem = OdometryLiftSystem(hwInit("odometryServoLeft"), hwInit("odometryServoCenter"))
    @JvmField val tseGrabber: TseGrabber = TseGrabber(hwInit("tseGrabberPivotServo"), hwInit("tseGrabberServo"))

    @JvmField val fullIntakeSystem = FullIntakeSystem(depositLiftMotor, depositServo, intakeMotor1, intakeMotor2)

    @JvmField val imuControllerC = IMUController(hardwareMap, id = 'C')
    override val holonomicRR: HolonomicRR = HolonomicRR(imuControllerC, frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor,
                                    TwoWheelOdometryLocalizer(carouselMotor, intakeMotor1, imuControllerC))

    init {
        intakeMotor1.direction = DcMotorSimple.Direction.REVERSE
    }

}