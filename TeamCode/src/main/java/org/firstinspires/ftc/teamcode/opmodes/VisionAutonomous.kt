package org.firstinspires.ftc.teamcode.opmodes

import android.graphics.Color
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.functions.StartingPosition.*
import org.firstinspires.ftc.teamcode.library.functions.PostAllianceHubTask.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem.DepositLiftPosition
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem.DepositLiftPosition.*
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.freightfrenzy.ColorMarkerVisionConstants
import org.firstinspires.ftc.teamcode.library.vision.freightfrenzy.ColorMarkerVisionPipeline
import java.lang.Math.PI

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Vision Autonomous", group = "Main")
class VisionAutonomous : BaseAutonomous<ExtThinBot>() {

    /*
        VARIABLES: Hardware and Control
     */

    /*
        VARIABLES: Menu Options
     */
    private var allianceColor: AllianceColor by config.custom("Alliance Color", RED, BLUE)
    private var startingPosition: StartingPosition by config.custom("Starting Position", NEAR_CAROUSEL, CENTER, NEAR_WAREHOUSE)
    private var depositPosition: DepositLiftPosition by config.custom("Default Deposit Position", LOW, MIDDLE, HIGH)
    private var postAllianceHubTask: PostAllianceHubTask by config.custom("Post- Alliance Hub Task", NOTHING, WAREHOUSE, CAROUSEL)
    private var extraDelayBeforeStart: Int by config.int("Delay Before First Action", 0, 0..20000 step 1000)
    private var extraDelayAfterShippingHub: Int by config.int("Delay After Shipping Hub", 0, 0..20000 step 1000)
    private var webcamScanningDuration: Int by config.int("Webcam Scanning Duration", 2000, 0..5000 step 500)
    private var cameraPosition: CameraPosition by config.custom("Camera Position", CameraPosition.LEFT, CameraPosition.CENTER, CameraPosition.RIGHT)

    override fun runOpMode() {
        robot = ExtThinBot(hardwareMap)
        super.autonomousConfig()
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
                    when (startingPosition) {
                        CENTER -> -12.5
                        NEAR_CAROUSEL -> -36.0
                        NEAR_WAREHOUSE -> 12.5
                    },
                    -63.0 reverseIf BLUE,
                    -PI / 2 reverseIf BLUE
            )

            robot.fullIntakeSystem.resetDepositZero()
            robot.fullIntakeSystem.update()

            robot.webcamServo.position = when (cameraPosition) {
                CameraPosition.LEFT -> 0.78
                CameraPosition.CENTER -> 0.79
                CameraPosition.RIGHT -> 0.8
            }
            sleep(webcamScanningDuration.toLong())

            val contourResult = cvContainer.pipeline.contourResult?.standardized
            if (contourResult != null) depositPosition = angledContourResult(contourResult)
            telem.addData("Found Deposit?", contourResult != null)
            telem.addData("Deposit Position", depositPosition)
            telem.update()

            sleep(extraDelayBeforeStart.toLong())

            builder(Math.PI/2 reverseIf BLUE)
                    .splineToConstantHeading(Vector2d(
                            -12.5,
                            (if (depositPosition == LOW) -46.0 else -43.1) reverseIf BLUE), Math.PI/2 reverseIf BLUE)
                    .buildAndRun(safeMode = true)
            if (robot.holonomicRR.safeModeLastTriggered != null) {
                telem.addLine("EMERGENCY STOP!!!")
                telem.addLine("Failed to register robot movement")
                telem.update()
                return
            }

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
                    builder(-PI/2 reverseIf BLUE)
                            .splineToConstantHeading(Vector2d(-66.0, -56.5 reverseIf BLUE), PI)
                            .buildAndRun()

                    //Turn Carousel
                    robot.carouselMotor.power = (0.45) reverseIf BLUE
                    sleep(6000)
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
                                    Vector2d(8.0, -65.0 reverseIf BLUE),
                                    0.0
                            )
                            .buildAndRun()
                    builder().strafeLeft(4.0 reverseIf RED).buildAndRun()
                    builder().forward(30.0).buildAndRun()
                }
            }
        }
    }

    private fun angledContourResult(contourResult: ColorMarkerVisionPipeline.ContourResult): DepositLiftPosition {
        val center = (contourResult.max.x + contourResult.min.x) / 2
        return when {
            center < ColorMarkerVisionConstants.BOUNDARY_FIRST -> LOW
            center < ColorMarkerVisionConstants.BOUNDARY_SECOND -> MIDDLE
            else -> HIGH
        }
    }

    private infix fun Double.reverseIf(testColor: AllianceColor) : Double = if (allianceColor == testColor) -this else this
}
