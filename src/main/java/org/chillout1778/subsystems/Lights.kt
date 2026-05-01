package org.chillout1778.subsystems

import com.ctre.phoenix6.controls.*
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.AnimationDirectionValue
import com.ctre.phoenix6.signals.LarsonBounceValue
import com.ctre.phoenix6.signals.RGBWColor
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.Robot

object Lights: SubsystemBase() {

    val candle = CANdle(Constants.CanIds.CANDLE)

    enum class Colors(val c: RGBWColor) {
        BLACK(RGBWColor(0,0,0)),
        WHITE(RGBWColor(255,255,255)),
        BLUE(RGBWColor(0,0,255)),
    }

    enum class LEDState {
        OFF,
        SHIFT_ACTIVE,
        SHIFT_INACTIVE,
        COUNTING_DOWN_ACTIVE, // 5-second countdown indicating END of ACTIVE shift
        COUNTING_DOWN_FINAL_TWO_SECONDS_ACTIVE, // same as above, but counting with double speed (two bounces per second)
        COUNTING_DOWN_INACTIVE, // 5-second countdown indicating END of INACTIVE shift
        COUNTING_DOWN_FINAL_TWO_SECONDS_INACTIVE, // same as above, but counting with double speed (2 bounces per second)
        AUTO_TRANSITION_ENDGAME,
        POST_MATCH,
        PRE_MATCH_DISABLED
    }

    var currentState = LEDState.OFF
    var lastState = LEDState.OFF

    fun updateLedsWithNewState() {
        resetAllAnimations()

        candle.setControl(
            when (currentState) {
                // Solid color states
                LEDState.OFF -> SolidColor(Constants.Lights.ROBOT_LED_ZONE.first, Constants.Lights.ROBOT_LED_ZONE.last).withColor(Colors.BLACK.c)
                LEDState.SHIFT_ACTIVE -> SolidColor(Constants.Lights.SHOOTER_INNER_EDGE_ZONE.first, Constants.Lights.SHOOTER_OUTER_EDGE_ZONE.last).withColor(Colors.BLUE.c)
                LEDState.SHIFT_INACTIVE -> SolidColor(Constants.Lights.SHOOTER_INNER_EDGE_ZONE.first, Constants.Lights.SHOOTER_OUTER_EDGE_ZONE.last).withColor(Colors.WHITE.c)
                LEDState.AUTO_TRANSITION_ENDGAME -> SolidColor(Constants.Lights.SHOOTER_INNER_EDGE_ZONE.first, Constants.Lights.SHOOTER_OUTER_EDGE_ZONE.last).withColor(Colors.BLUE.c)

                // Disabled states
                LEDState.POST_MATCH -> RainbowAnimation(
                    Constants.Lights.SHOOTER_INNER_EDGE_ZONE.first,
                    Constants.Lights.SHOOTER_OUTER_EDGE_ZONE.last)
                    .withBrightness(Constants.Lights.LED_BRIGHTNESS)
                    .withDirection(AnimationDirectionValue.Forward)
                    .withSlot(0)
                LEDState.PRE_MATCH_DISABLED -> LarsonAnimation(
                    Constants.Lights.SHOOTER_INNER_EDGE_ZONE.first,
                    Constants.Lights.SHOOTER_OUTER_EDGE_ZONE.last)
                    .withColor(Colors.WHITE.c)
                    .withBounceMode(LarsonBounceValue.Front)
                    .withFrameRate(80.0)
                    .withSlot(0)

                // Timer states
                LEDState.COUNTING_DOWN_ACTIVE -> StrobeAnimation(
                    Constants.Lights.SHOOTER_INNER_EDGE_ZONE.first,
                    Constants.Lights.SHOOTER_OUTER_EDGE_ZONE.last)
                    .withColor(Colors.BLUE.c)
                    .withFrameRate(2.0) // We want to turn the LEDs on then off (two frames) once a second. So 2 frames per second = 2Hz
                    .withSlot(0)
                LEDState.COUNTING_DOWN_FINAL_TWO_SECONDS_ACTIVE -> StrobeAnimation(
                    Constants.Lights.SHOOTER_INNER_EDGE_ZONE.first,
                    Constants.Lights.SHOOTER_OUTER_EDGE_ZONE.last)
                    .withColor(Colors.BLUE.c)
                    .withFrameRate(4.0) // We want to double our flashing speed for the last 2 seconds. This is when we should start shooting, in an ideal situation
                    .withSlot(0)
                LEDState.COUNTING_DOWN_INACTIVE -> StrobeAnimation(
                    Constants.Lights.SHOOTER_INNER_EDGE_ZONE.first,
                    Constants.Lights.SHOOTER_OUTER_EDGE_ZONE.last)
                    .withColor(Colors.WHITE.c)
                    .withFrameRate(2.0) // We want to turn the LEDs on then off (two frames) once a second. So 2 frames per second = 2Hz
                    .withSlot(0)
                LEDState.COUNTING_DOWN_FINAL_TWO_SECONDS_INACTIVE -> StrobeAnimation(
                    Constants.Lights.SHOOTER_INNER_EDGE_ZONE.first,
                    Constants.Lights.SHOOTER_OUTER_EDGE_ZONE.last)
                    .withColor(Colors.WHITE.c)
                    .withFrameRate(4.0) // We want to double our flashing speed for the last 2 seconds. This is when we should start shooting, in an ideal situation
                    .withSlot(0)
            }
        )
    }


    var wasEnabled = false

    val lightsTimer = Timer()

    fun resetTimer() {
        lightsTimer.reset()
        lightsTimer.start()
    }

    init {
        resetTimer()
    }

    override fun periodic() {
        if (Robot.isDisabled) {
            // At the end of a match
            if (wasEnabled) {
                // If we haven't started the rainbow, start it and a timer
                if (currentState != LEDState.POST_MATCH) {
                    currentState = LEDState.POST_MATCH
                    resetTimer()
                // If the rainbow has been running for longer than 5 seconds, stop it
                } else if (lightsTimer.get() > 5.0) {
                    currentState = LEDState.OFF
                    wasEnabled = false
                    resetTimer()
                }
            } else {
                // Every 10 seconds, start the larson
                if (lightsTimer.get() > 10.0) {
                    currentState = LEDState.PRE_MATCH_DISABLED
                    resetTimer()
                }

                // If the larson is running and it has completed a full cycle of the robot, stop it
                if (currentState == LEDState.PRE_MATCH_DISABLED && lightsTimer.get() > 0.45) {
                    currentState = LEDState.OFF
                    resetTimer()
                }
            }
        } else {
            wasEnabled = true

            if (Robot.autoIsRunning) {
                currentState = LEDState.SHIFT_ACTIVE
            } else {
                val matchTimer = Robot.matchTimer.get()

                if (matchTimer < Constants.Lights.SHIFT_1_COUNTDOWN_TIMESTAMP) {
                    currentState = LEDState.SHIFT_ACTIVE
                } else if (matchTimer < (Constants.Lights.SHIFT_1_COUNTDOWN_TIMESTAMP + 3)) {
                    currentState = LEDState.COUNTING_DOWN_ACTIVE
                } else if (matchTimer < (Constants.Lights.SHIFT_1_COUNTDOWN_TIMESTAMP + 5)) {
                    currentState = LEDState.COUNTING_DOWN_FINAL_TWO_SECONDS_ACTIVE
                } else if (matchTimer < Constants.Lights.SHIFT_2_COUNTDOWN_TIMESTAMP) {
                    setStateSolid()
                } else if (matchTimer < (Constants.Lights.SHIFT_2_COUNTDOWN_TIMESTAMP + 3)) {
                    setStateCountdownSlow()
                } else if (matchTimer < (Constants.Lights.SHIFT_2_COUNTDOWN_TIMESTAMP + 5)) {
                    setStateCountdownFast()
                } else if (matchTimer < Constants.Lights.SHIFT_3_COUNTDOWN_TIMESTAMP) {
                    setStateSolid()
                } else if (matchTimer < (Constants.Lights.SHIFT_3_COUNTDOWN_TIMESTAMP + 3)) {
                    setStateCountdownSlow()
                } else if (matchTimer < (Constants.Lights.SHIFT_3_COUNTDOWN_TIMESTAMP + 5)) {
                    setStateCountdownFast()
                } else if (matchTimer < (Constants.Lights.SHIFT_4_COUNTDOWN_TIMESTAMP)) {
                    setStateSolid()
                } else if (matchTimer < (Constants.Lights.SHIFT_4_COUNTDOWN_TIMESTAMP + 3)) {
                    setStateCountdownSlow()
                } else if (matchTimer < (Constants.Lights.SHIFT_4_COUNTDOWN_TIMESTAMP + 5)) {
                    setStateCountdownFast()
                } else if (matchTimer < Constants.Lights.ENDGAME_COUNTDOWN_TIMESTAMP) {
                    setStateSolid()
                } else if (matchTimer < (Constants.Lights.ENDGAME_COUNTDOWN_TIMESTAMP + 3)) {
                    setStateCountdownSlow()
                } else if (matchTimer < (Constants.Lights.ENDGAME_COUNTDOWN_TIMESTAMP + 5)) {
                    setStateCountdownFast()
                } else {
                    // We're in endgame, so set to active (blue)
                    currentState = LEDState.SHIFT_ACTIVE
                }
            }
        }

        if (currentState != lastState) {
            lastState = currentState
            updateLedsWithNewState()
        }
    }

    // This function clears all 8 animation slots from the candle
    private fun resetAllAnimations() {
        for (i in 0..7) {
            candle.setControl(EmptyAnimation(i))
        }
    }

    private fun setStateSolid() {
        currentState = if (Robot.isOurHubEnabledNow()) LEDState.SHIFT_ACTIVE else LEDState.SHIFT_INACTIVE
    }

    private fun setStateCountdownSlow() {
        currentState = if (Robot.isOurHubEnabledNow()) LEDState.COUNTING_DOWN_ACTIVE else LEDState.COUNTING_DOWN_INACTIVE
    }

    private fun setStateCountdownFast() {
        currentState = if (Robot.isOurHubEnabledNow()) LEDState.COUNTING_DOWN_FINAL_TWO_SECONDS_ACTIVE else LEDState.COUNTING_DOWN_FINAL_TWO_SECONDS_INACTIVE
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addBooleanProperty("Red alliance won auto", { Robot.isRedAllianceAutoWinner }, { Robot.isRedAllianceAutoWinner = it })
        builder.addBooleanProperty("Alliance hub enabled", {Robot.isOurHubEnabledNow()}, {})
        builder.addDoubleProperty("Teleop enabled timer", {Robot.matchTimer.get()}, {})
        builder.addIntegerProperty("Shift number", {Robot.allianceShiftNumber.toLong()}, {})
    }
}