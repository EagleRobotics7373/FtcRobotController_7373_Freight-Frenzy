package org.firstinspires.ftc.teamcode.testopmodes.visiontests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.DashboardVar
import org.firstinspires.ftc.teamcode.library.functions.StartingPosition
import org.firstinspires.ftc.teamcode.library.functions.rangeClip
import org.firstinspires.ftc.teamcode.library.robot.systems.meet2.FullIntakeSystem
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.freightfrenzy.ColorMarkerComparisonVisionPipeline
import org.firstinspires.ftc.teamcode.library.vision.freightfrenzy.ColorMarkerVisionConstants

@TeleOp(name="OpenCV: ColorMarkerComparisonVisionTest", group="Vision")
class ColorMarkerComparisonVisionTestOpMode: LinearOpMode() {

    init {
        Thread.sleep(100)
        this.telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    }

    var useStandardized by DashboardVar(false, "useStandardized", this::class)
    var webcamServoPosition by DashboardVar(0.76, "webcamPosition", this::class)
    var startingPositionCenter by DashboardVar(false, "startingPositionCenter", this::class)

    override fun runOpMode() {

        val webcamServo = hardwareMap.servo.get("webcamServo")
        val voltageSensor = hardwareMap.voltageSensor.first()

        val gamepad1Ex = GamepadEx(gamepad1)

        val cvContainer = VisionFactory.createOpenCv(
            hardwareMap,
            "Webcam 1",
            ColorMarkerComparisonVisionPipeline())
        cvContainer.start()

        waitForStart()

        cvContainer.pipeline.tracking = true
        cvContainer.pipeline.shouldKeepTracking = true

        while (opModeIsActive()) {
            gamepad1Ex.readButtons()

            val contourResult = cvContainer.pipeline.tseContourResult?.let {
                if (useStandardized) it.standardized else it
            }
            val firstMarker = cvContainer.pipeline.firstMarker
            val depositPosition = when (startingPositionCenter) {
                true -> { // we are looking diagonally at right stack
                    if (contourResult == null) FullIntakeSystem.DepositLiftPosition.HIGH // right
                    else if (firstMarker != null && contourResult.min.x < firstMarker.min.x) FullIntakeSystem.DepositLiftPosition.LOW
                    else FullIntakeSystem.DepositLiftPosition.MIDDLE
                }
                else -> { // we are looking straight on at the stack
                    if (contourResult == null) FullIntakeSystem.DepositLiftPosition.LOW // left
                    else if (firstMarker != null && contourResult.min.x < firstMarker.min.x) FullIntakeSystem.DepositLiftPosition.MIDDLE
                    else FullIntakeSystem.DepositLiftPosition.HIGH
                }
            }
            telemetry.addData("Deposit Position", depositPosition)
            telemetry.addData("Standardized", useStandardized)
            telemetry.addLine()

            if (contourResult == null) {
                telemetry.addData("Detected", "No")
            } else {
                telemetry.addData("Position", cvContainer.pipeline.positionResult)
                telemetry.addData("Detected", "Yes")
                telemetry.addData("Min", contourResult.min.toString())
                telemetry.addData("Max", contourResult.min.toString())
                telemetry.addData("Area", contourResult.area.toString())
                telemetry.addData("Ratio", contourResult.ratio.toString())
                telemetry.addData("Width", contourResult.width.toString())

                val centerX = (contourResult.min.x + contourResult.max.x)/2
                telemetry.addData("Center X", centerX)

                if (useStandardized) {
                    telemetry.addData("Field position (by center x)", when (centerX) {
                        in 0.0..ColorMarkerVisionConstants.BOUNDARY_FIRST -> "LEFT"
                        in ColorMarkerVisionConstants.BOUNDARY_FIRST..ColorMarkerVisionConstants.BOUNDARY_SECOND -> "CENTER"
                        else -> "RIGHT"
                    })
                }
            }

            when {
                gamepad1.a -> useStandardized = true
                gamepad1.b -> useStandardized = false
            }

            when {
                gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) -> webcamServoPosition -= 0.08
                gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP) -> webcamServoPosition += 0.08
                gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) -> webcamServoPosition -= 0.04
                gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) -> webcamServoPosition += 0.04
            }
            webcamServoPosition = webcamServoPosition.rangeClip(0.0, 1.0)
            webcamServo.position = webcamServoPosition
            telemetry.addData("Webcam Servo Position", webcamServoPosition)

            if (voltageSensor != null) telemetry.addData("Voltage", voltageSensor.voltage)

            telemetry.update()
        }


    }
}