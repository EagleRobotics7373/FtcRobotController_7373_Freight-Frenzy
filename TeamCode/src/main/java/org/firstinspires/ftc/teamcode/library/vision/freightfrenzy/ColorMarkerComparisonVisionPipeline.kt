package org.firstinspires.ftc.teamcode.library.vision.freightfrenzy

import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor
import org.firstinspires.ftc.teamcode.library.vision.base.ImageResolution
import org.firstinspires.ftc.teamcode.library.vision.base.ResolutionPipeline
import org.firstinspires.ftc.teamcode.library.vision.base.coerceIn
import org.firstinspires.ftc.teamcode.library.vision.base.times
import org.firstinspires.ftc.teamcode.library.vision.freightfrenzy.ColorMarkerVisionConstants.*

import org.opencv.core.*
import org.opencv.core.Core.*
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.*


class ColorMarkerComparisonVisionPipeline() : ResolutionPipeline() {

    // Public variable allowing OpModes to access contour details
    var contourResult: ContourResult? = null
    var positionResult: MarkerPosition? = null
    var allianceColor: AllianceColor = AllianceColor.RED

    // The mat that we will analyze, alone with the channel number we need to extract from HSV
    private var hlsMat: Mat = Mat()
    private var tseThresholdResult: Mat = Mat()
    private var markersThresholdResult: Mat = Mat()

    override var resolution : ImageResolution = ImageResolution.R_720x480

    override fun processFrame(input: Mat): Mat {
        if (tracking) {

            // Blur the image for greater contour recognition accuracy
            blur(input, input, Size(5.0, 10.0))

            // Convert our input, in RGB format, to HLS (hue, luminosity, saturation)
            cvtColor(input, hlsMat, COLOR_RGB2HLS)

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
                    tseThresholdResult                     // resultant mat
            )

            when (allianceColor) {
                AllianceColor.RED -> inRange(
                        hlsMat,                             // original mat
                        Scalar(                             // lower bound for threshold
                                RED_HUE_LOW,
                                COLOR_LUM_LOW,
                                RED_SAT_LOW),
                        Scalar(                             // upper bound for threshold
                                RED_HUE_HIGH,
                                COLOR_LUM_HIGH,
                                CONTOUR_SAT_UPPER_BOUND),
                        markersThresholdResult              // resultant mat
                )
                AllianceColor.BLUE -> inRange(
                        hlsMat,                             // original mat
                        Scalar(                             // lower bound for threshold
                                CONTOUR_HUE_LOWER_BOUND,
                                CONTOUR_LUM_LOWER_BOUND,
                                CONTOUR_SAT_LOWER_BOUND),
                        Scalar(                             // upper bound for threshold
                                CONTOUR_HUE_UPPER_BOUND,
                                CONTOUR_LUM_UPPER_BOUND,
                                CONTOUR_SAT_UPPER_BOUND),
                        markersThresholdResult              // resultant mat
                )
            }

            val elementType = Imgproc.CV_SHAPE_RECT;
            val kernelSize = CONTOUR_DILATION_KSIZE;
            val element = getStructuringElement(
                    elementType, Size(2 * kernelSize + 1, 2 * kernelSize + 1),
                    Point(kernelSize, kernelSize)
            )
            dilate(
                    tseThresholdResult,                    // original mat
                    tseThresholdResult,                    // resultant mat - just overwrite the original
                    element                             // iterations - more of these means more erosion
            )

            val drawingMat = when(CONTOUR_MAT_PRINTOUT_NUM) {
                0       -> input
                1       -> hlsMat
                2       -> tseThresholdResult
                else    -> markersThresholdResult
            }

            // TSE Contour
            val contoursCmpltd = findContours(tseThresholdResult, drawingMat)
            val resCntur = contoursCmpltd
                .filter { it.width > CONTOUR_ENTITY_MINWIDTH * resolution.scale && it.min.y < input.rows() * (0.70) }
                .maxByOrNull { it.area }
            this.contourResult = resCntur

            // Marker contours
            val markerContoursCmpltd = findContours(markersThresholdResult, drawingMat)
            val resCnturs = markerContoursCmpltd
                    .filter { it.width > CONTOUR_MARKER_MINWIDTH * resolution.scale
                            && it.width < CONTOUR_MARKER_MAXWIDTH * resolution.scale
                            && it.min.y < input.rows() * (0.70) }
                    .sortedBy { it.min.x }

            positionResult = if (resCntur != null && resCnturs.size >= 2) {
                val firstMarker = resCnturs[0]
                val secondMarker = resCnturs[1]
                when {
                    resCntur.min.x < firstMarker.min.x && resCntur.min.x < secondMarker.min.x -> MarkerPosition.LEFT
                    firstMarker.min.x < resCntur.min.x && resCntur.min.x < secondMarker.min.x -> MarkerPosition.CENTER
                    else -> MarkerPosition.RIGHT
                }
            } else null



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
            Point(min.x / this@ColorMarkerComparisonVisionPipeline.resolution.width,
                min.y / this@ColorMarkerComparisonVisionPipeline.resolution.height),
            Point(max.x / this@ColorMarkerComparisonVisionPipeline.resolution.width,
                max.y / this@ColorMarkerComparisonVisionPipeline.resolution.height),
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

    private fun findContours(mat: Mat, drawingMat: Mat): List<ColorMarkerComparisonVisionPipeline.ContourResult> {
        val contours = emptyList<MatOfPoint>().toMutableList()
        val hierarchy = Mat()
        findContours(
                mat,
                contours,
                hierarchy,
                RETR_EXTERNAL,
                CHAIN_APPROX_SIMPLE
        )

        return contours.mapIndexed { index, matOfPoint ->
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

    }

    enum class MarkerPosition {
        LEFT, CENTER, RIGHT
    }


}