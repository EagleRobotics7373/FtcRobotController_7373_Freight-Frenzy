package org.firstinspires.ftc.teamcode.testopmodes.visiontests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.ValueProvider
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.freightfrenzy.ColorMarkerVisionPipeline

@TeleOp(name="OpenCV: ColorMarkerVisionTest", group="Vision")
class ColorMarkerVisionTestOpMode: LinearOpMode() {

    var useStandardized = false

    override fun runOpMode() {

        FtcDashboard.getInstance().addConfigVariable(
            this::class.simpleName,
            "Standardized?",
            object: ValueProvider<Boolean> {
                override fun get(): Boolean = useStandardized

                override fun set(value: Boolean?) {
                    if (value != null) {
                        useStandardized = value
                    }
                }
            },
        true)

        val cvContainer = VisionFactory.createOpenCv(
            hardwareMap,
            "Webcam 1",
            ColorMarkerVisionPipeline())
        cvContainer.start()

        waitForStart()

        cvContainer.pipeline.tracking = true
        cvContainer.pipeline.shouldKeepTracking = true

        while (opModeIsActive()) {
            val contourResult = cvContainer.pipeline.contourResult?.let {
                if (useStandardized) it.standardized else it
            }

            telemetry.addData("Standardized", useStandardized)
            telemetry.addLine()

            if (contourResult == null) {
                telemetry.addData("Detected", "No")
            } else {
                telemetry.addData("Detected", "Yes")
                telemetry.addData("Min", contourResult.min.toString())
                telemetry.addData("Max", contourResult.min.toString())
                telemetry.addData("Area", contourResult.area.toString())
                telemetry.addData("Ratio", contourResult.ratio.toString())
                telemetry.addData("Width", contourResult.width.toString())
            }

            when {
                gamepad1.a -> useStandardized = true
                gamepad1.b -> useStandardized = false
            }

            telemetry.update()
        }


    }
}