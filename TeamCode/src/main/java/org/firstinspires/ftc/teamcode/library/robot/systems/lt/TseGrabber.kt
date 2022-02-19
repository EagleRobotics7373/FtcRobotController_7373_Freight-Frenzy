package org.firstinspires.ftc.teamcode.library.robot.systems.lt

import com.qualcomm.robotcore.hardware.Servo

class TseGrabber
constructor (private val pivotServo : Servo,
             private val grabServo  : Servo,
             private val wristServo : Servo,
             private val reverse    : Boolean = false)
{

    /**
    Enum class defining positions for the arm to pivot
     */
    enum class PivotPosition(val position: Double) {
        GRAB(0.14),
        RELEASE(0.5),
        RELEASE_HIGHER(0.55),
        STORAGE(0.84),
    }

    /**
    Enum class defining positions for the grabber to pivot
     */
    enum class GrabPosition(val position: Double) {
        GRAB(0.00),
        MID_GRAB(0.35),
        STORAGE(0.14)
    }

    /**
    Enum class defining positions for the wrist to pivit
     */
    enum class WristPosition(val position: Double) {
        GROUND(0.14),
        HIGH(0.54)
    }

    /**
     * Pivot the tse grabber arm to a specified position
     * @param loc the position to pivot
     */
    private fun pivot(loc: PivotPosition) { pivotServo.position = loc.position.oneMinus() }

    /**
     * Pivot the tse grabber end effector to a specified position
     * @param loc the position to pivot the grabber
     */
    private fun grab(loc: GrabPosition) { grabServo.position = loc.position }

    /**
     * Pivot the tse grabber wrist effector to a specified position
     * @param loc the position to pivot the wrist
     */
    private fun wrist(loc: WristPosition) { wristServo.position = loc.position }

    fun move(grab: GrabPosition? = null, pivot: PivotPosition? = null, wrist: WristPosition? = null) {
        state = TseGrabberState.CUSTOM
        if (grab != null)  grab(grab)
        if (pivot != null) pivot(pivot)
        if (wrist != null) wrist(wrist)
    }

    /**
    Describes various states for the tse grabber for easier gamepad-based control
     */
    enum class TseGrabberState(
            val action: ((TseGrabber, Long)->Unit)?,
            val prev: ()->TseGrabberState?,
            val next: ()->TseGrabberState?)
    {

        STORAGE(
                action = { it, _ -> it.move(GrabPosition.STORAGE, PivotPosition.STORAGE, WristPosition.GROUND) },
                prev = { RELEASE },
                next = { GRAB_PREP }),
        GRAB_PREP(
                action = { it, _ -> it.move(GrabPosition.MID_GRAB, PivotPosition.GRAB, WristPosition.GROUND) },
                prev = { STORAGE },
                next = { GRAB }),
        GRAB(
                action = { it, _ -> it.move(GrabPosition.GRAB, PivotPosition.GRAB, WristPosition.GROUND) },
                prev = { GRAB_PREP },
                next = { IN_AIR }),
        IN_AIR(
                action = { it, _ -> it.move(GrabPosition.GRAB, PivotPosition.RELEASE, WristPosition.HIGH) },
                prev = { GRAB },
                next = { RELEASE }),
        RELEASE(
                action = { it, _ -> it.move(GrabPosition.MID_GRAB, PivotPosition.RELEASE, WristPosition.HIGH) },
                prev = { IN_AIR },
                next = { STORAGE }),
        CUSTOM(
                action = null,
                prev = { STORAGE },
                next = { STORAGE })

    }

    /**
     * Records the current state of the tse grabber
     */
    var state: TseGrabberState = TseGrabberState.CUSTOM
        set(value) { stateBeginTime = System.currentTimeMillis(); value.action?.invoke(this, 0); field = value; }

    /**
     * Records the time at which a state is changed
     */
    var stateBeginTime: Long = System.currentTimeMillis()

    /**
     * Reports the duration in which a certain state has been active
     * Subtracts the result of [System.currentTimeMillis] from [stateBeginTime]
     */
    val stateDuration: Long get() = System.currentTimeMillis() - stateBeginTime

    /**
     * Quick-access function for going to the next state
     */
    fun prevState() { state = state.prev() ?: state }

    /**
     * Quick-access function for returning to the previous state
     */
    fun nextState() { state = state.next() ?: state }

    /**
     * Re-run the state action function to allow for time-delayed actions
     */
    fun update() { state.action?.invoke(this, stateDuration) }

    private fun Double.oneMinus() = if (reverse) (1-this) else this

}