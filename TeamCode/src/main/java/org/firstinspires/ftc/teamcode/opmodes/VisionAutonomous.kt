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
    private var postAllianceHubTask: PostAllianceHubTask by config.custom("Post Alliance Hub Task", NOTHING, WAREHOUSE, CAROUSEL)

    override fun runOpMode() {
        robot = ExtThinBot(hardwareMap)
        val cvContainer = VisionFactory.createOpenCv(
                hardwareMap,
                "Webcam 1",
                ColorMarkerVisionPipeline())
        cvContainer.start()

        waitForStart()

        cvContainer.pipeline.tracking = true
        cvContainer.pipeline.shouldKeepTracking = true
        super.operateMenu()

        while (opModeIsActive()) {
            val contourResult = cvContainer.pipeline.contourResult?.standardized
            if (contourResult != null) {
                val center = (contourResult.max.x + contourResult.min.x) / 2
                depositPosition =
                        (if (center < 0.33) LOW
                        else if (center < 0.66) MIDDLE
                        else HIGH)
            }

            robot.holonomicRR.poseEstimate = Pose2d(
                    -12.5,
                    (-63.0) reverseIf(BLUE),
                    (Math.PI / 2) reverseIf(BLUE) //startingHeading
            )

            builder()
                    .splineToConstantHeading(Vector2d(-12.5, -43.1 reverseIf BLUE), Math.PI/2 reverseIf BLUE)
                    .buildAndRun()

            //Drop Off Pre-Load
            robot.fullIntakeSystem.depositLiftAuto(depositPosition, 1.0)
            robot.depositServo.position = 0.32

            builder()
                    .strafeTo(Vector2d(-63.0, (-53.0) reverseIf(BLUE)))
                    .buildAndRun()
            when (postAllianceHubTask) {
                NOTHING -> {}
                CAROUSEL -> {
                    builder()
                            .strafeTo(Vector2d(-63.0, -53.0 reverseIf BLUE))
                            .buildAndRun()

                    //Turn Carousel
                    robot.carouselMotor.velocity = (0.5) reverseIf(BLUE)
                    sleep(1000)
                    robot.carouselMotor.velocity = 0.0

                    builder()
                            .splineToConstantHeading(
                                    Vector2d(-62.0, -36.0 reverseIf BLUE),
                                    Math.PI / 2 reverseIf BLUE
                            )
                            .buildAndRun()
                }
                WAREHOUSE -> {
                    builder(0.0)
                            .splineToConstantHeading(
                                    Vector2d(12.0, -63.0 reverseIf BLUE),
                                    0.0
                            )
                            .strafeTo(Vector2d(40.0, -63.0 reverseIf BLUE))
                            .buildAndRun()
                }
            }
        }
    }

    private infix fun Double.reverseIf(testColor: AllianceColor) : Double = if (allianceColor == testColor) -this else this
}
