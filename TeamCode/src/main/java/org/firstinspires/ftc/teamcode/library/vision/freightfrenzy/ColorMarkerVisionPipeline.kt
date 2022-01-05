package org.firstinspires.ftc.teamcode.library.vision.freightfrenzy

import org.firstinspires.ftc.teamcode.library.vision.base.ImageResolution
import org.firstinspires.ftc.teamcode.library.vision.base.ResolutionPipeline
import org.firstinspires.ftc.teamcode.library.vision.base.coerceIn
import org.firstinspires.ftc.teamcode.library.vision.base.times
import org.firstinspires.ftc.teamcode.library.vision.freightfrenzy.ColorMarkerVisionConstants.*

import org.opencv.core.*
import org.opencv.core.Core.*
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.*


class ColorMarkerVisionPipeline() : ResolutionPipeline() {

    // Public variable allowing OpModes to access contour details
    var contourResult: ContourResult? = null

    // The mat that we will analyze, alone with the channel number we need to extract from HSV
    private var subMat: Mat = Mat()
    private var hlsMat: Mat = Mat()
    private var thresholdResult: Mat = Mat()

    override var resolution : ImageResolution = ImageResolution.R_720x480

    override fun processFrame(input: Mat): Mat {
        if (tracking) {

            // Blur the image for greater contour recognition accuracy
            blur(input, input, Size(5.0, 10.0))

            subMat = input.submat((CUTOFF_TOP * input.rows()).toInt(), (CUTOFF_BOTTOM * input.rows()).toInt(), 0, input.cols())

            // Convert our input, in RGB format, to HLS (hue, luminosity, saturation)
            cvtColor(subMat, hlsMat, COLOR_RGB2HLS)

            // Threshold the HLS mat to only include objects within desired HLS range
            inRange(
                    hlsMat,                             // original mat
                    Scalar(                             // lower bound for threshold
                            CONTOUR_HUE_LOWER_BOUND,
                            CONTOUR_LUM_LOWER_BOUND,
                            CONTOUR_SAT_LOWER_BOUND),
                    Scalar(                             // upper bound for threshold
                            CONTOUR_HUE_UPPER_BOUND,
                            CONTOUR_LUM_UPPER_BOUND,
                            CONTOUR_SAT_UPPER_BOUND),
                    thresholdResult                     // resultant mat
            )

            val elementType = Imgproc.CV_SHAPE_RECT;
            val kernelSize = CONTOUR_DILATION_KSIZE;
            val element = getStructuringElement(
                    elementType, Size(2 * kernelSize + 1, 2 * kernelSize + 1),
                    Point(kernelSize, kernelSize)
            )
            dilate(
                    thresholdResult,                    // original mat
                    thresholdResult,                    // resultant mat - just overwrite the original
                    element                             // iterations - more of these means more erosion
            )

            val drawingMat = when(MAT_OUTPUT_NUM) {
                0       -> input
                1       -> hlsMat
                else    -> thresholdResult
            }
            val contours = emptyList<MatOfPoint>().toMutableList()
            val hierarchy = Mat()
            findContours(
                    thresholdResult,
                    contours,
                    hierarchy,
                    RETR_EXTERNAL,
                    CHAIN_APPROX_SIMPLE
            )

            val contoursCmpltd = contours.mapIndexed { index, matOfPoint ->

                drawContours(
                        drawingMat,
                        contours,
                        index,
                        Scalar.all(150.0),
                        5
                )

                var minX = Int.MAX_VALUE
                var minY = Int.MAX_VALUE
                var maxX = Int.MIN_VALUE
                var maxY = Int.MIN_VALUE
                for (point in matOfPoint.toArray()) {
                    if (point.x < minX) minX = point.x.toInt()
                    if (point.y < minY) minY = point.y.toInt()
                    if (point.x > maxX) maxX = point.x.toInt()
                    if (point.y > maxY) maxY = point.y.toInt()
                }

                println("CONTOUR $index min=($minX, $minY) max=($maxX, $maxY)")

                return@mapIndexed ContourResult(
                        Point(minX.toDouble(), minY.toDouble()),
                        Point(maxX.toDouble(), maxY.toDouble()))
            }

            val resCntur = contoursCmpltd
                .filter { it.width > CONTOUR_ENTITY_MINWIDTH * resolution.scale && it.min.y < input.rows() * (0.70) }
                .maxByOrNull { it.area }

            this.contourResult = resCntur

            addLabels(drawingMat)
            return drawingMat
        }
        else {
            addLabels(input)
            return input
        }
    }

    inner class ContourResult
    constructor(val min: Point, val max: Point)
    {
        val width = max.x - min.x
        val height = max.y - min.y
        val ratio = width.toDouble() / height
        val area = width * height

        val standardized: ContourResult
        get() = ContourResult(
            Point(min.x / this@ColorMarkerVisionPipeline.resolution.width,
                min.y / this@ColorMarkerVisionPipeline.resolution.height),
            Point(max.x / this@ColorMarkerVisionPipeline.resolution.width,
                max.y / this@ColorMarkerVisionPipeline.resolution.height),
        )
    }

    private fun inverseColorAtPoint(mat: Mat, point: Point): DoubleArray {
        val newPoint = point.coerceIn(mat)
        return mat.get(newPoint.y.toInt(), newPoint.x.toInt()) // Get color at this point
                .map { 255 - it }                              // For each channel, invert the color
//                .dropLast(1)
                .toDoubleArray()                               // Re-convert to DoubleArray
    }


    private fun addLabels(mat: Mat) {

        // Place text representing the team name (at the top)
        val teamNameStartPoint = Point(5.0, 30.0)
                .times(resolution.scale)
                .coerceIn(mat)
        putText(mat, "Eagle Robotics Team 7373", teamNameStartPoint,
                FONT_HERSHEY_SIMPLEX, 1 * resolution.scale,
                Scalar(inverseColorAtPoint(mat, teamNameStartPoint)), 2)

        // Place text indicating the detector purpose (below the team name)
        val detectorNameStartPoint = Point(5.0, 50.0)
                .times(resolution.scale)
                .coerceIn(mat)
        putText(mat, "FREIGHT FRENZY ColorMarkerVision Pipeline", detectorNameStartPoint,
                FONT_HERSHEY_SIMPLEX, 0.55 * resolution.scale,
                Scalar(inverseColorAtPoint(mat, detectorNameStartPoint)), 2)

        // Below code left in-place in case specfic result should be shown on camera feed
//        putText(mat, resultText,
//                resultTextStartPoint,
//                FONT_HERSHEY_SIMPLEX, 0.6 * resolution.scale,
//                Scalar(inverseColorAtPoint(mat, resultTextStartPoint)), 2)
    }




}