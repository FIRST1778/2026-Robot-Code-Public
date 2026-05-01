package org.chillout1778.subsystems

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.Controls
import kotlin.math.sign

object Intake : SubsystemBase() {

    var extensionTarget = 0.0
    var rollerVoltage = 9.0

    val extensionMainMotor = TalonFX(Constants.CanIds.INTAKE_EXTENSION_MAIN_MOTOR)
    val extensionFollowerMotor = TalonFX(Constants.CanIds.INTAKE_EXTENSION_FOLLOWER_MOTOR)
    val rollerMotor = TalonFX(Constants.CanIds.INTAKE_ROLLER_MOTOR)

    var extensionPid = Constants.Intake.EXTENSION_PID_COEFFICIENTS.createPid()

    enum class State(val extensionPosition: Double, val rollerVoltage: Double) {
        OUT_INTAKE(Constants.Intake.MAX_EXTENSION_DISTANCE, 13.0),
        RETRACTED(0.0, 0.0),
        OUT_OFF(Constants.Intake.MAX_EXTENSION_DISTANCE, 0.0),
        SHOOT_SHAKE_IN(Constants.Intake.SHOOT_SHAKE_POS, 13.0),
    }

    init {
        extensionMainMotor.configurator.apply { Constants.Intake.EXTENSION_CONFIG }
        extensionFollowerMotor.configurator.apply { Constants.Intake.EXTENSION_CONFIG }
        rollerMotor.configurator.apply { Constants.Intake.ROLLER_CONFIG }

        extensionMainMotor.setPosition(0.0)

        extensionFollowerMotor.setControl(Follower(extensionMainMotor.deviceID, MotorAlignmentValue.Opposed))

        extensionMainMotor.optimizeBusUtilization()
        extensionFollowerMotor.optimizeBusUtilization()
        rollerMotor.optimizeBusUtilization()

        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            extensionMainMotor.position, extensionMainMotor.velocity, extensionMainMotor.motorVoltage,
            extensionFollowerMotor.position, extensionFollowerMotor.velocity, extensionFollowerMotor.motorVoltage,
            rollerMotor.position, extensionFollowerMotor.velocity, rollerMotor.motorVoltage )
        BaseStatusSignal.setUpdateFrequencyForAll(10.0,
            extensionMainMotor.supplyCurrent, extensionMainMotor.supplyVoltage,
            extensionFollowerMotor.supplyCurrent, extensionFollowerMotor.supplyVoltage,
            rollerMotor.supplyCurrent, extensionFollowerMotor.supplyVoltage)
    }

    val rollerSupplyCurrent : Double get() = rollerMotor.supplyCurrent.valueAsDouble

    private var state = State.RETRACTED

    override fun periodic() {
        state = if (Controls.controls.wantIntakeOut) {
            if (Controls.controls.wantIntakeSpinning)
                State.OUT_INTAKE
            else State.OUT_OFF
        } else {
            State.RETRACTED
        }

        if (Controls.controls.wantIntakeShootRetract) state = State.SHOOT_SHAKE_IN

        extensionPid.setpoint = state.extensionPosition
        rollerMotor.setVoltage(if (Controls.controls.wantIntakeReversed) -state.rollerVoltage else state.rollerVoltage)

        extensionMainMotor.setVoltage(
            extensionPid.calculate(extensionMainMotor.position.valueAsDouble)
                + sign(extensionPid.setpoint - (extensionMainMotor.position.valueAsDouble)) * Constants.Intake.EXTENSION_PID_COEFFICIENTS.kS
        )
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("kP", { Constants.Intake.EXTENSION_PID_COEFFICIENTS.kP }, {
            Constants.Intake.EXTENSION_PID_COEFFICIENTS.kP = it;
            extensionPid.p = it
        })
        builder.addDoubleProperty("kD", { Constants.Intake.EXTENSION_PID_COEFFICIENTS.kD }, {
            Constants.Intake.EXTENSION_PID_COEFFICIENTS.kD = it;
            extensionPid.d = it
        })
        builder.addDoubleProperty("kS", { Constants.Intake.EXTENSION_PID_COEFFICIENTS.kS }, {
            Constants.Intake.EXTENSION_PID_COEFFICIENTS.kS = it;
        })

        builder.addDoubleProperty("Intake target", { extensionTarget }, { extensionTarget = it })
        builder.addDoubleProperty("Roller voltage", { rollerVoltage }, { rollerVoltage = it })
        builder.addDoubleProperty("Intake current pose", { extensionMainMotor.position.valueAsDouble },{})
        builder.addStringProperty("Intake state", { state.toString() }, {})

        builder.addDoubleProperty("Intake Rollers Supply Current", { rollerMotor.supplyCurrent.valueAsDouble}, {})
        builder.addDoubleProperty("Intake Extension 16 Supply Current",{ extensionMainMotor.supplyCurrent.valueAsDouble}, {})
        builder.addDoubleProperty("Intake Extension 15 Supply Current", { extensionFollowerMotor.supplyCurrent.valueAsDouble}, {})

        builder.addDoubleProperty("Intake Rollers Supply Voltage", { rollerMotor.supplyVoltage.valueAsDouble },{})
        builder.addDoubleProperty("Intake Extension 16 Supply Voltage", { extensionMainMotor.supplyVoltage.valueAsDouble },{})
        builder.addDoubleProperty("Intake Extension 15 Supply Voltage", { extensionFollowerMotor.supplyVoltage.valueAsDouble },{})
    }
}