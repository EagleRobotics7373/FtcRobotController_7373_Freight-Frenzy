package org.firstinspires.ftc.teamcode.library.vision.base

import org.firstinspires.ftc.teamcode.library.functions.Stoppable
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline

class OpenCvContainer<out Pipeline : OpenCvPipeline>(
    val camera: OpenCvCamera,
    val pipeline: Pipeline,
    private val resolution: ImageResolution,
    private val rotation: OpenCvCameraRotation = OpenCvCameraRotation.UPRIGHT) : Stoppable {

    init {
        camera.setPipeline(pipeline)
        if (pipeline is ResolutionPipeline) pipeline.resolution = resolution
        camera.showFpsMeterOnViewport(false)
    }

    fun start() {
        camera.openCameraDeviceAsync(object: OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(resolution.width, resolution.height, rotation)
            }

            override fun onError(errorCode: Int) { return }
        })

        CameraStreamSplitter.startCameraStream(camera)
    }

    override fun stop() {
        camera.stopStreaming()
        camera.closeCameraDeviceAsync {  }
        CameraStreamSplitter.stopCameraStream()
    }
}