package org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.TelemetryMenu
import kotlin.reflect.KMutableProperty

class ReflectiveTelemetryMenu constructor(val telemetry: Telemetry, vararg items: ReflectiveMenuItem) : TelemetryMenu {
    private val list = items.toList()
    var current = list[0]

    init {
        refresh()
    }

    fun refresh() {
        list.forEach {
            telemetry.addData((if (it === current) "-> " else "") + it.description,
                    (if (it.canIterateBackward()) " << " else "")
                            + it
                            + (if (it.canIterateForward()) " >> " else ""))
        }
        telemetry.update()

    }

    override fun nextItem() {
        val currentNum = list.indexOf(current)
        if (currentNum < list.size-1) current = list[currentNum+1]
        refresh()
    }
    override fun previousItem() {
        val currentNum = list.indexOf(current)
        if (currentNum > 0) current = list[currentNum-1]
        refresh()
    }
    override fun iterateForward() {
        if (current.canIterateForward())
            current.iterateForward()
        refresh()
    }
    override fun iterateBackward() {
        if (current.canIterateBackward())
            current.iterateBackward()
        refresh()
    }
}

abstract class ReflectiveMenuItem
constructor(val description: String) {
    abstract override fun toString(): String
    abstract fun iterateForward()
    abstract fun iterateBackward(): Unit
    abstract fun canIterateForward(): Boolean
    abstract fun canIterateBackward(): Boolean
}

class ReflectiveMenuItemInteger
constructor(description: String,
            private val property: KMutableProperty<Int>,
            private val lowerLimit: Int,
            private val upperLimit: Int,
            private val incrementBy: Int)
    : ReflectiveMenuItem(description) {
    override fun toString(): String = property.getter.call().toString()
    override fun iterateForward() {
        if (canIterateForward())
            property.setter.call(property.getter.call() + incrementBy)
    }

    override fun iterateBackward() {
        if (canIterateBackward())
            property.setter.call(property.getter.call() - incrementBy)
    }

    override fun canIterateForward(): Boolean =
            property.getter.call() + incrementBy <= upperLimit

    override fun canIterateBackward(): Boolean =
            property.getter.call() - incrementBy >= lowerLimit

}

class ReflectiveMenuItemBoolean
constructor(description: String,
            private val property: KMutableProperty<Boolean>
) : ReflectiveMenuItem(description) {
    override fun toString(): String = property.getter.call().toString()
    override fun iterateForward() {
        if (canIterateForward())
            property.setter.call(true)
    }

    override fun iterateBackward() {
        if (canIterateBackward())
            property.setter.call(false)
    }

    override fun canIterateForward(): Boolean =
            !property.getter.call()

    override fun canIterateBackward(): Boolean =
            property.getter.call()
}

class ReflectiveMenuItemEnum<T>
constructor(description: String,
            private val property: KMutableProperty<T>,
            vararg val allowedElements: T,
            val toStringMethod : (T)->String = {it.toString()})
    : ReflectiveMenuItem(description) {

    var num = allowedElements.indexOf(property.getter.call())

    init {
        if (num == -1) num = 0
    }

    override fun toString(): String =
            toStringMethod(property.getter.call())

    override fun iterateForward() {
        if (canIterateForward()) property.setter.call(allowedElements[++num])
    }

    override fun iterateBackward() {
        if (canIterateBackward()) property.setter.call(allowedElements[--num])
    }

    override fun canIterateForward(): Boolean =
            num < allowedElements.size-1


    override fun canIterateBackward(): Boolean =
            num > 0
}

class ReflectiveMenuItemFeedback
constructor(description: String,
            val action: ()->String)
    : ReflectiveMenuItem(description) {
    override fun toString(): String {
        return action()
    }

    override fun iterateForward() {
    }

    override fun iterateBackward() {
    }

    override fun canIterateForward(): Boolean {
        return false
    }

    override fun canIterateBackward(): Boolean {
        return false
    }
}