package org.firstinspires.ftc.teamcode.library.functions

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.ValueProvider
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KClass
import kotlin.reflect.KProperty

class DashboardVar<T>(
    initialValue: T,
    private val name: String,
    private val clazz: KClass<*>,
    private val validator: ((T)->Boolean)? = null
) : ReadWriteProperty<Any?, T> {

    private var storedValue = initialValue

    init {
        FtcDashboard.getInstance().addConfigVariable(
            clazz.simpleName,
            name,
            object: ValueProvider<T> {
                override fun get(): T { return storedValue }
                override fun set(value: T) { if (validator?.invoke(value) != false) storedValue = value }
            },
            true
        )
    }

    override fun getValue(thisRef: Any?, property: KProperty<*>): T {
        return storedValue
    }

    override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
        if (validator?.invoke(value) != false) storedValue = value
        println("DashboardVar received value update for " +
                "${clazz.qualifiedName}:${clazz.simpleName}.${name} " +
                "within property ${property.name}")
    }

}

class DashboardVarWrapped<T>(
    property: ReadWriteProperty<Any?, T>,
    private val name: String,
    private val clazz: KClass<*>
): ReadWriteProperty<Any?, T> {

    var storedValue: T by property

    init {
        FtcDashboard.getInstance().addConfigVariable(
            clazz.simpleName,
            name,
            object: ValueProvider<T> {
                override fun get(): T { return storedValue }
                override fun set(value: T) { storedValue = value }
            },
            true
        )
    }

    override fun getValue(thisRef: Any?, property: KProperty<*>): T {
        return storedValue
    }

    override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
        storedValue = value
        println("DashboardVar received value update for " +
                "${clazz.qualifiedName}:${clazz.simpleName}.${name} " +
                "within property ${property.name}")
    }
}