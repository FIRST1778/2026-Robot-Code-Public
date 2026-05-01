package org.chillout1778.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj2.command.Command
import org.chillout1778.Constants
import org.chillout1778.Controls
import org.chillout1778.subsystems.Swerve
import java.util.function.Supplier

class TeleOpDriveCommand(
    private val driveInputsSupplier: Supplier<Controls.Inputs>
): Command() {
    init {
        addRequirements(Swerve)
    }

    val drive = SwerveRequest.FieldCentric()
        .withDeadband(Constants.Swerve.MAX_SPEED * 0.1)
        .withRotationalDeadband(Constants.Swerve.MAX_SPEED * 0.1)
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
    val brake = SwerveRequest.SwerveDriveBrake()

    override fun execute() {
        val inputs = driveInputsSupplier.get()

        Swerve.setControl(
            if (driveInputsSupplier.get().wantSwerveBrake) brake
            else
                drive.withVelocityX(inputs.forward * Constants.Swerve.MAX_SPEED)
            .withVelocityY(inputs.left * Constants.Swerve.MAX_SPEED)
            .withRotationalRate(inputs.rotation * Constants.Swerve.MAX_ANGULAR_RATE))
    }
}