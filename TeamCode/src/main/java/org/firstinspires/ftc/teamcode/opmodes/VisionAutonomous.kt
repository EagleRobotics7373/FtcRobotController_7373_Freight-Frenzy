package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.Companion.persistingAllianceColor
import org.firstinspires.ftc.teamcode.library.functions.StartingPosition.*
import org.firstinspires.ftc.teamcode.library.functions.PostAllianceHubTask.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtThinBot
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem.DepositLiftPosition
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem.DepositLiftPosition.*
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.freightfrenzy.ColorMarkerComparisonVisionPipeline
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
    private var postAllianceHubTask: PostAllianceHubTask by config.custom("Post- Alliance Hub Task", BACKPEDAL, WAREHOUSE, CAROUSEL, NOTHING)
    private var extraDelayBeforeStart: Int by config.int("Delay Before First Action", 0, 0..20000 step 1000)
    private var extraDelayAfterShippingHub: Int by config.int("Delay After Shipping Hub", 0, 0..20000 step 1000)
    private var webcamScanningDuration: Int by config.int("Webcam Scanning Duration", 2000, 0..5000 step 500)
    private var safeModeErrorThreshold: Int by config.int("Safe Mode Error Threshold", 10, 0..30 step 2)

    override fun runOpMode() {
        persistingAllianceColor = allianceColor
        robot = ExtThinBot(hardwareMap)
        robot.webcamServo.position = 0.1
        super.autonomousConfig()
        robot.holonomicRR.safeModeErrorThreshold = safeModeErrorThreshold
        val cvContainer = VisionFactory.createOpenCv(
                hardwareMap,
                "Webcam 1",
                ColorMarkerComparisonVisionPipeline())
        cvContainer.start()

        operateMenu {
            setWebcamServoPosition()
            if (gamepad2.x) robot.fullIntakeSystem.depositLiftManual(-0.10)
            if (gamepad2.y) robot.fullIntakeSystem.resetDepositZero()
        }

        if (opModeIsActive()) {
            cvContainer.pipeline.allianceColor = allianceColor
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

            setWebcamServoPosition()
            sleep(webcamScanningDuration.toLong())

//            val contourResult = cvContainer.pipeline.contourResult?.standardized
//            if (contourResult != null) depositPosition = angledContourResult(contourResult)
//            telem.addData("Found Deposit?", contourResult != null)
            val contourResult = cvContainer.pipeline.tseContourResult
            val firstMarker = cvContainer.pipeline.firstMarker
            depositPosition = when (startingPosition) {
                CENTER -> { // we are looking diagonally at right stack
                    if (contourResult == null) HIGH // left
                    else if (firstMarker != null && contourResult.min.x < firstMarker.min.x) LOW
                    else MIDDLE
                }
                else -> { // we are looking straight on at the stack
                    if (contourResult == null) LOW // left
                    else if (firstMarker != null && contourResult.min.x < firstMarker.min.x) MIDDLE
                    else HIGH
                }
            }
            telem.addData("Deposit Position for ${cvContainer.pipeline.allianceColor} Alliance", depositPosition)
            telem.update()

            sleep(extraDelayBeforeStart.toLong())

            builder(Math.PI/2 reverseIf BLUE)
                    .splineToConstantHeading(Vector2d(
                            -12.5,
                            (if (depositPosition == LOW) -45.5 else -44.0) reverseIf BLUE), Math.PI/2 reverseIf BLUE)
                    .buildAndRun(safeMode = true)
            if (robot.holonomicRR.safeModeLastTriggered != null) {
                telem.addLine("EMERGENCY STOP!!!")
                telem.addLine("Failed to register robot movement")
                telem.update()
                return
            }

            //Drop Off Pre-Load
            robot.fullIntakeSystem.depositLiftAuto(depositPosition, 0.6)
            if (depositPosition != LOW) forDurationMs(2000) { robot.fullIntakeSystem.update() }
            robot.fullIntakeSystem.depositServoIsExtended = true
            sleep(1500)
            robot.fullIntakeSystem.depositServoIsExtended = false

            sleep(extraDelayAfterShippingHub.toLong())
            robot.fullIntakeSystem.depositLiftAuto(LOW, 0.6)
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
                    robot.carouselMotorSystem.carouselMotor.power = (0.45) reverseIf BLUE
                    sleep(6000)
                    robot.carouselMotorSystem.carouselMotor.velocity = 0.0

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
                BACKPEDAL -> {
                    builder()
                            .strafeTo(Vector2d(robot.holonomicRR.poseEstimate.x, -63.0 reverseIf BLUE))
                            .buildAndRun()
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

    private fun setWebcamServoPosition() {
        robot.webcamServo.position = when (startingPosition) {
            CENTER -> 0.45
            else -> 0.55
        }
    }

    private infix fun Double.reverseIf(testColor: AllianceColor) : Double = if (allianceColor == testColor) -this else this
}
