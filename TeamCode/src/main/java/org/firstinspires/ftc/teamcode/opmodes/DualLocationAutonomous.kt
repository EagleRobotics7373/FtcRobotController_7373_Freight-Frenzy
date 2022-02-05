package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.functions.AutonomousObjective.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Main")
class DualLocationAutonomous : BaseAutonomous<ExtThinBot>() {

    /*
        VARIABLES: Hardware and Control
     */


//    private lateinit var cvContainer     : OpenCvContainer<ColorMarkerVisionPipeline>

//    private lateinit var player          : ExtDirMusicPlayer

    /*
        VARIABLES: Menu Options
     */
    private var allianceColor: AllianceColor by config.custom("Alliance Color", RED, BLUE)
    private var autonomousObjective: AutonomousObjective by config.custom("Starting Location", WAREHOUSE, CAROUSEL)

    override fun runOpMode() {
        robot = ExtThinBot(hardwareMap)
//        cvContainer = VisionFactory.createOpenCv(hardwareMap, "Webcam 1", ColorMarkerVisionPipeline())
//        cvContainer.pipeline.shouldKeepTracking = true
//        cvContainer.pipeline.tracking = true
        robot.carouselMotorSystem.carouselMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        super.operateMenu(null)

        when (autonomousObjective) {
            WAREHOUSE -> {
                robot.holonomic.runUsingEncoder(0.0, 26.0, 1.0)
                when (allianceColor) {
                    RED -> robot.holonomic.runUsingEncoder(24.0, 0.0, 1.0)
                    BLUE -> robot.holonomic.runUsingEncoder(-24.0, 0.0, 1.0)
                }
            }
            CAROUSEL -> {
                robot.holonomic.runUsingEncoder(0.0, 2.0, 0.25)
                when (allianceColor) {
                    RED -> {
                        robot.carouselMotorSystem.carouselMotor.velocity = 5.0 * 145.1
                        sleep(1000)
                        robot.carouselMotorSystem.carouselMotor.velocity = 0.0
                        robot.holonomic.runUsingEncoder(-26.0, 0.0, 1.0)
                    }
                    BLUE -> {
                        robot.carouselMotorSystem.carouselMotor.velocity = -5.0 * 145.1
                        sleep(1000)
                        robot.carouselMotorSystem.carouselMotor.velocity = 0.0
                        robot.holonomic.runUsingEncoder(26.0, 0.0, 1.0)
                    }
                }
                robot.holonomic.runUsingEncoder(0.0, 10.0, 1.0)
            }
        }

        robot.holonomic.stop()
        robot.carouselMotorSystem.carouselMotor.velocity = 0.0

    }


}
