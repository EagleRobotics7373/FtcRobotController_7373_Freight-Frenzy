package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.functions.PostAllianceHubTask.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem.DepositLiftPosition
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem.DepositLiftPosition.*
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.freightfrenzy.ColorMarkerVisionPipeline

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Vision Autonomous", group = "Main")
class VisionAutonomous : BaseAutonomous<ExtThinBot>() {

    /*
        VARIABLES: Hardware and Control
     */

    /*
        VARIABLES: Menu Options
     */
    private var allianceColor: AllianceColor by config.custom("Alliance Color", RED, BLUE)
    private var depositPosition: DepositLiftPosition by config.custom("Default Deposit Position", LOW, MIDDLE, HIGH)
    private var postAllianceHubTask: PostAllianceHubTask by config.custom("Post- Alliance Hub Task", NOTHING, WAREHOUSE, CAROUSEL)
    private var extraDelayBeforeStart: Int by config.int("Delay Before First Action", 0, 0..20000 step 1000)
    private var extraDelayAfterShippingHub: Int by config.int("Delay After Shipping Hub", 0, 0..20000 step 1000)
    private var webcamScanningDuration: Int by config.int("Webcam Scanning Duration", 0, 0..5000 step 500)

    override fun runOpMode() {
        robot = ExtThinBot(hardwareMap)
        val cvContainer = VisionFactory.createOpenCv(
                hardwareMap,
                "Webcam 1",
                ColorMarkerVisionPipeline())
        cvContainer.start()

        super.operateMenu()

        if (opModeIsActive()) {
            cvContainer.pipeline.tracking = true
            cvContainer.pipeline.shouldKeepTracking = true

            robot.holonomicRR.poseEstimate = Pose2d(
                    -12.5,
                    (-63.0) reverseIf BLUE ,
                    -(Math.PI / 2) reverseIf BLUE //startingHeading
            )

            robot.fullIntakeSystem.resetDepositZero()
            robot.fullIntakeSystem.update()

            robot.webcamServo.position = 0.10
            sleep(webcamScanningDuration.toLong())

            val contourResult = cvContainer.pipeline.contourResult?.standardized
            if (contourResult != null) depositPosition = angledContourResult(contourResult)

            sleep(extraDelayBeforeStart.toLong())

            builder(Math.PI/2 reverseIf BLUE)
                    .splineToConstantHeading(Vector2d(
                            -12.5,
                            (if (depositPosition == LOW) -46.0 else -43.1) reverseIf BLUE), Math.PI/2 reverseIf BLUE)
                    .buildAndRun()

            //Drop Off Pre-Load
            robot.fullIntakeSystem.depositLiftAuto(depositPosition, 0.6)
            forDurationMs(2000) { robot.fullIntakeSystem.update() }
            robot.fullIntakeSystem.depositServoIsExtended = true
            sleep(800)
            robot.fullIntakeSystem.depositServoIsExtended = false

            sleep(extraDelayAfterShippingHub.toLong())

            when (postAllianceHubTask) {
                NOTHING -> {
                    builder()
                            .forward(2.0)
                            .buildAndRun()
                }
                CAROUSEL -> {
                    builder()
                            .strafeTo(Vector2d(-63.0, -53.5 reverseIf BLUE))
                            .buildAndRun()

                    //Turn Carousel
                    robot.carouselMotor.power = (0.3) reverseIf BLUE
                    sleep(5000)
                    robot.carouselMotor.velocity = 0.0

                    builder(Math.PI/2 reverseIf BLUE)
                            .splineToConstantHeading(
                                    Vector2d(-62.0, -36.0 reverseIf BLUE),
                                    Math.PI / 2 reverseIf BLUE
                            )
                            .buildAndRun()
                }
                WAREHOUSE -> {
                    builder(-Math.PI/2 reverseIf BLUE)
                            .splineTo(
                                    Vector2d(12.0, -63.0 reverseIf BLUE),
                                    0.0
                            )
                            .strafeTo(Vector2d(40.0, -63.0 reverseIf BLUE))
                            .buildAndRun()
                }
            }
        }
    }

    private fun angledContourResult(contourResult: ColorMarkerVisionPipeline.ContourResult): DepositLiftPosition {
        val center = (contourResult.max.x + contourResult.min.x) / 2
        return when {
            center < 0.33 -> LOW
            center < 0.66 -> MIDDLE
            else -> HIGH
        }
    }

    private infix fun Double.reverseIf(testColor: AllianceColor) : Double = if (allianceColor == testColor) -this else this
}
