package org.chillout1778.subsystems

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.Controls
import kotlin.math.abs

object Indexer : SubsystemBase() {

    val spindexerMotor = TalonFX(Constants.CanIds.SPINDEXER_MOTOR)

    val ballTunnelMotor = TalonFX(Constants.CanIds.BALL_TUNNEL_MOTOR)

    val unjamTimer = Timer() // This is the timer to determine how long to spin the indexer in reverse to unjam
    var jammed = false
    val jammedTimer = Timer() // This times how long the spindexer needs to be stopped for it to consider itself jammed

    enum class IndexerState(val ballTunnelVolts: Double, var spindexerVolts: Double) {
        STOP(0.0, 0.0),
        START(12.0, 6.0)
    }

    init {
        spindexerMotor.configurator.apply { Constants.Indexer.SPINDEXER_CONFIG }
        ballTunnelMotor.configurator.apply { Constants.Indexer.BALL_TUNNEL_CONFIG }

        spindexerMotor.optimizeBusUtilization()
        ballTunnelMotor.optimizeBusUtilization()

        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            spindexerMotor.position, spindexerMotor.velocity, spindexerMotor.motorVoltage,
            ballTunnelMotor.position, ballTunnelMotor.velocity, ballTunnelMotor.motorVoltage)
        BaseStatusSignal.setUpdateFrequencyForAll(10.0,
            spindexerMotor.supplyCurrent, spindexerMotor.supplyVoltage,
            ballTunnelMotor.supplyCurrent, ballTunnelMotor.supplyVoltage)
    }

    var state = IndexerState.STOP

    val ballTunnelPid = Constants.Indexer.BALL_TUNNEL_PID_COEFFICIENTS.createPid()
    val spindexerPid = Constants.Indexer.SPINDEXER_PID_COEFFICIENTS.createPid()

    override fun periodic() {
        if (Constants.TUNING_MODE) {
            spindexerMotor.setVoltage(state.spindexerVolts)
            ballTunnelMotor.setVoltage(state.ballTunnelVolts)

            return
        }

        var shouldReverseBallTunnel = Controls.controls.wantReverseSpindexer

        // JAM DETECTION, INITIAL CODE FROM MI ROBOTICS 5937
        if (state == IndexerState.START) {
            if (jammedTimer.isRunning && jammedTimer.hasElapsed(0.25)) { // If the spindexer has been stopped for 0.4 seconds, we can safely assume it is jammed
                jammed = true
                unjamTimer.start()
                jammedTimer.stop()
                jammedTimer.reset()
            }

            // TODO: Tune this time (MI reverses for 0.1 second, so we can probably reduce it a decent amount)
            if (jammed && unjamTimer.isRunning && unjamTimer.hasElapsed(0.2)) { // If we're jammed, we want to reverse the spindexer and ball tunnel for 0.5 seconds. After that time, resume to normal operation
                jammed = false
                unjamTimer.stop()
                unjamTimer.reset()
            }

            if (abs(spindexerMotor.velocity.valueAsDouble) < 4.0 && !jammed) { // If our indexer isn't spinning right now, and it isn't known to be jammed, start a timer
                jammedTimer.start()
            } else if (jammedTimer.isRunning) { // If we're not marked as jammed (even if we were), that means we unjammed without needing to reverse. Stop the timer
                jammedTimer.stop()
                jammedTimer.reset()
            }

            // Now, if the spindexer is marked as 'jammed', we need to reverse it.
            if (jammed) {
                shouldReverseBallTunnel = true
            }
        }

        ballTunnelMotor.setVoltage(if (shouldReverseBallTunnel) -state.ballTunnelVolts else state.ballTunnelVolts)
        spindexerMotor.setVoltage(if (shouldReverseBallTunnel) -state.spindexerVolts else state.spindexerVolts)
    }

    val spindexerSupplyCurrent: Double get() = spindexerMotor.supplyCurrent.valueAsDouble

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("Ball tunnel kV", { Constants.Indexer.BALL_TUNNEL_PID_COEFFICIENTS.kV }, {Constants.Indexer.BALL_TUNNEL_PID_COEFFICIENTS.kV = it})
        builder.addDoubleProperty("Ball tunnel kS", { Constants.Indexer.BALL_TUNNEL_PID_COEFFICIENTS.kS }, {Constants.Indexer.BALL_TUNNEL_PID_COEFFICIENTS.kS = it})
        builder.addDoubleProperty("Spindexer kV", { Constants.Indexer.SPINDEXER_PID_COEFFICIENTS.kV }, { Constants.Indexer.SPINDEXER_PID_COEFFICIENTS.kV = it })
        builder.addDoubleProperty("Spindexer kS", { Constants.Indexer.SPINDEXER_PID_COEFFICIENTS.kS }, { Constants.Indexer.SPINDEXER_PID_COEFFICIENTS.kS = it })

        builder.addDoubleProperty("Ball Tunnel Supply Current", { ballTunnelMotor.supplyCurrent.valueAsDouble}, {})
        builder.addDoubleProperty("Spindexer Supply Current", { spindexerMotor.supplyCurrent.valueAsDouble}, {})

        builder.addDoubleProperty("Ball Tunnel Supply Voltage", { ballTunnelMotor.supplyVoltage.valueAsDouble}, {})
        builder.addDoubleProperty("Spindexer Supply Voltage", { spindexerMotor.supplyVoltage.valueAsDouble}, {})

        builder.addStringProperty("Unjam timer", { "Is running: " + unjamTimer.isRunning + " & elapsed time: " + unjamTimer.get() }, { })
        builder.addStringProperty("Jammed timer", { "Is running: " + jammedTimer.isRunning + " & elapsed time: " + jammedTimer.get() }, { })
        builder.addBooleanProperty("Is jammed", { jammed }, { })

        builder.addDoubleProperty("Spindexer volts", { IndexerState.START.spindexerVolts}, { IndexerState.START.spindexerVolts = it })
        builder.addDoubleProperty("Spindexer rps", { spindexerMotor.velocity.valueAsDouble}, {})
    }
}