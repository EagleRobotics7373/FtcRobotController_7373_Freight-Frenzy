package org.firstinspires.ftc.teamcode.library.vision.base

import org.opencv.core.Mat
import org.opencv.core.Point

fun Point.times(increment: Double): Point {
    return Point(this.x.times(increment), this.y.times(increment))
}

fun Point.coerceIn(mat: Mat): Point {
    return Point(this.x.coerceIn(0.0, mat.cols()-1.0), this.y.coerceIn(0.0, mat.rows()-1.0))
}

operator fun Point.plus(other: Point): Point {
    return Point(this.x + other.x, this.y + other.y)
}

operator fun Point.minus(other: Point): Point {
    return Point(this.x - other.x, this.y - other.y)
}