package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Estacionar", group = "Main")
class ParkingAutonomous : BaseAutonomous<ExtThinBot>() {

    /*
        VARIABLES: Hardware and Control
     */

    /*
        VARIABLES: Menu Options
     */
//    private var allianceColor: AllianceColor by config.custom("Alliance Color", RED, BLUE)
    private var powerX: Int by config.int("Drivetrain Power (x)", 0, -100..100 step 5)
    private var powerY: Int by config.int("Drivetrain Power (y)", 0, -100..100 step 5)
    private var drivingTime: Int by config.int("Time (ms)", 0, 0..5000 step 100)

    override fun runOpMode() {
        robot = ExtThinBot(hardwareMap)
//        cvContainer = VisionFactory.createOpenCv(hardwareMap, "Webcam 1", ColorMarkerVisionPipeline())
//        cvContainer.pipeline.shouldKeepTracking = true
//        cvContainer.pipeline.tracking = true
        robot.carouselMotorSystem.carouselMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        super.operateMenu(null)

        //Mecanum Drive

        //Mecanum Drive
        val vertical = powerY * 0.01
        val horizontal = powerX * 0.01
        val pivot = 0.0


//        robot.holonomic.runWithoutEncoderVectored(horizontal, vertical, pivot, 0);
        robot.frontRightMotor.power = pivot - vertical + horizontal
        robot.backRightMotor.power = pivot - vertical - horizontal
        robot.frontLeftMotor.power = pivot + vertical + horizontal
        robot.backLeftMotor.power = pivot + vertical - horizontal

        sleep(drivingTime.toLong())

        robot.holonomic.stop()
        robot.carouselMotorSystem.carouselMotor.velocity = 0.0

    }


}
