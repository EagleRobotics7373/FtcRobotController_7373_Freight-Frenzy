package org.firstinspires.ftc.teamcode.library.vision.freightfrenzy

import org.firstinspires.ftc.teamcode.library.functions.AllianceColor
import org.firstinspires.ftc.teamcode.library.functions.DashboardVar
import org.firstinspires.ftc.teamcode.library.vision.base.*
import org.firstinspires.ftc.teamcode.library.vision.freightfrenzy.ColorMarkerVisionConstants.*

import org.opencv.core.*
import org.opencv.core.Core.*
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.*


class ColorMarkerComparisonVisionPipeline() : ResolutionPipeline() {

    // Public variable allowing OpModes to access contour details
    var tseContourResult: ContourResult? = null; private set
    var firstMarker: ContourResult? = null; private set
    var secondMarker: ContourResult? = null; private set

    var positionResult: MarkerPosition? = null
    var allianceColor: AllianceColor by DashboardVar(AllianceColor.RED, "Alliance Color", this::class)

    // The mat that we will analyze, alone with the channel number we need to extract from HSV
    private var hlsMat: Mat = Mat()
    private var tseThresholdResult: Mat = Mat()
    private var markersThresholdResult: Mat = Mat()

    override var resolution : ImageResolution = ImageResolution.R_720x480

    override fun processFrame(input: Mat): Mat {
        if (tracking) {

            /*
            Initial preparation
             */

            // Blur the image for greater contour recognition accuracy
            blur(input, input, Size(5.0, 10.0))

            // Convert our input, in RGB format, to HLS (hue, luminosity, saturation)
            cvtColor(input, hlsMat, COLOR_RGB2HLS)

            // Set the active mat to draw on
            val drawingMat = when(MAT_OUTPUT_NUM) {
                0       -> input
                1       -> hlsMat
                2       -> tseThresholdResult
                else    -> markersThresholdResult
            }

            /*
            TSE image manipulation
             */

            // Threshold the HLS mat for TSE (only include objects within desired HLS range)
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
            val tseElementType = Imgproc.CV_SHAPE_RECT;
            val tseKernelSize = CONTOUR_DILATION_KSIZE;
            val tseElement = getStructuringElement(
                    tseElementType, Size(2 * tseKernelSize + 1, 2 * tseKernelSize + 1),
                    Point(tseKernelSize, tseKernelSize)
            )
            dilate(
                    tseThresholdResult,                    // original mat
                    tseThresholdResult,                    // resultant mat - just overwrite the original
                    tseElement                             // iterations - more of these means more erosion
            )

            // Get all possible TSE contours
            val possibleTseContours = findAndDrawContours(tseThresholdResult, drawingMat, SHOW_CONTOUR_TSE)

            // Filter and sort the list to get most likely TSE contour, set it to our instance variable contourResult
            this.tseContourResult = possibleTseContours
                    .filter { it.width > CONTOUR_ENTITY_MINWIDTH * resolution.scale && it.min.y < input.rows() * (0.70) }
                    .maxByOrNull { it.area }
            tseContourResult?.let { labelItem(it, "TSE CONTOUR", drawingMat) }

            /*
            Alliance color marker image manipulation
             */

            // Threshold the HLS mat for alliance color markers (only include objects within desired HLS range)
            // Will produce a result for red or blue alliance color, based on preset
            when (allianceColor) {
                AllianceColor.RED -> inRange(
                        hlsMat,                             // original mat
                        Scalar(                             // lower bound for threshold
                                COLOR_RED_HUE_LOW,
                                COLOR_LUM_LOW,
                                COLOR_RED_SAT_LOW),
                        Scalar(                             // upper bound for threshold
                                COLOR_RED_HUE_HIGH,
                                COLOR_LUM_HIGH,
                                COLOR_RED_SAT_HIGH),
                        markersThresholdResult              // resultant mat
                )
                AllianceColor.BLUE -> inRange(
                        hlsMat,                             // original mat
                        Scalar(                             // lower bound for threshold
                                COLOR_BLUE_HUE_LOW,
                                COLOR_LUM_LOW,
                                COLOR_BLUE_SAT_LOW),
                        Scalar(                             // upper bound for threshold
                                COLOR_BLUE_HUE_HIGH,
                                COLOR_LUM_HIGH,
                                COLOR_BLUE_SAT_HIGH),
                        markersThresholdResult              // resultant mat
                )
            }
            val markerElementType = Imgproc.CV_SHAPE_RECT;
            val markerKernelSize = COLOR_DILATION_KSIZE;
            val markerElement = getStructuringElement(
                    markerElementType, Size(2 * markerKernelSize + 1, 2 * markerKernelSize + 1),
                    Point(markerKernelSize, markerKernelSize)
            )
            dilate(
                    markersThresholdResult,                 // original mat
                    markersThresholdResult,                 // resultant mat - just overwrite the original
                    markerElement                           // iterations - more of these means more erosion
            )

            // Get all possible marker contours
            val possibleMarkerContours = findAndDrawContours(markersThresholdResult, drawingMat, SHOW_CONTOUR_MARKERS)

            // Filter and sort the list to get most likely marker contours
            val organizedMarkerContours = possibleMarkerContours
                    .filter { it.width > COLOR_WIDTH_MIN * resolution.scale     // width greater than minimum
                            && it.width < COLOR_WIDTH_MAX * resolution.scale    // width less than maximum
                            && it.min.y < input.rows() * (0.70) }                       // y position in upper 70% of image
                    .sortedBy { it.min.x }                                              // sort by x position (left to right)

            // Get the first two markers in the list, if available, and set them to our instance variables
            this.firstMarker = organizedMarkerContours.getOrNull(0)
            this.secondMarker = organizedMarkerContours.getOrNull(1)

            firstMarker?.let { labelItem(it, "FIRST MARKER", drawingMat) }
            secondMarker?.let { labelItem(it, "SECOND MARKER", drawingMat) }

            /*
            Determination based on what we found above
             */
            positionResult = if (tseContourResult != null && firstMarker != null && secondMarker != null) {
                when {
                    tseContourResult!!.min.x < firstMarker!!.min.x && tseContourResult!!.min.x < secondMarker!!.min.x -> MarkerPosition.LEFT
                    firstMarker!!.min.x < tseContourResult!!.min.x && tseContourResult!!.min.x < secondMarker!!.min.x -> MarkerPosition.CENTER
                    else -> MarkerPosition.RIGHT
                }
            } else null

            // Label everything, then return the mat that we labeled
            addGenericText(drawingMat)
            return drawingMat
        }
        else {
            addGenericText(input)
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


    private fun addGenericText(mat: Mat) {

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

    private fun labelItem(contourResult: ContourResult, name: String, drawingMat: Mat) {
        val textStartPoint = (contourResult.min - Point(1.0, 1.0)).coerceIn(drawingMat)
        rectangle(drawingMat, contourResult.min, contourResult.max, Scalar(inverseColorAtPoint(drawingMat, contourResult.min)), 2)
        putText(drawingMat, name, textStartPoint,
                FONT_HERSHEY_SIMPLEX, 0.5 * resolution.scale,
                Scalar(inverseColorAtPoint(drawingMat, textStartPoint)), 2)
    }

    private fun findAndDrawContours(mat: Mat, drawingMat: Mat, draw: Boolean = true): List<ColorMarkerComparisonVisionPipeline.ContourResult> {
        val contours = emptyList<MatOfPoint>().toMutableList()
        val hierarchy = Mat()
        findContours(
                mat,
                contours,
                hierarchy,
                RETR_EXTERNAL,
                CHAIN_APPROX_SIMPLE
        )


        return contours.mapIndexedNotNull { index, matOfPoint ->

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

            val contourResult = ContourResult(
                    Point(minX.toDouble(), minY.toDouble()),
                    Point(maxX.toDouble(), maxY.toDouble()))

            if (contourResult.width < 5) return@mapIndexedNotNull null

            if (draw) drawContours(
                    drawingMat,
                    contours,
                    index,
                    Scalar.all(150.0),
                    5
            )
            println("CONTOUR $index min=($minX, $minY) max=($maxX, $maxY)")

            return@mapIndexedNotNull contourResult
        }

    }

    enum class MarkerPosition {
        LEFT, CENTER, RIGHT
    }


}