package org.sert2521.offseason2025.commands

import edu.wpi.first.wpilibj2.command.*
import org.sert2521.offseason2025.ElevatorConstants
import org.sert2521.offseason2025.subsystems.DispenserSubsystem
import org.sert2521.offseason2025.subsystems.ElevatorSubsystem
import org.sert2521.offseason2025.subsystems.WristSubsystem
import org.sert2521.offseason2025.subsystems.drivetrain.Drivetrain


object ManipulatorRoutines {
    enum class ManipulatorPositions {
        STOW,
        L1,
        L2,
        L3,
        L4
    }

    fun intake(): Command {
        return ElevatorSubsystem.setHeightSafeCommand(ElevatorConstants.intake.elevatorGoalMeters)
            .alongWith(Commands.runOnce({ DispenserSubsystem.reverseOuttake = false }))
            .andThen(WristSubsystem.setAngleCommand(ElevatorConstants.intake.wristGoalRotations))
    }

    fun l1(): Command {
        return ElevatorSubsystem.setHeightSafeCommand(ElevatorConstants.l1.elevatorGoalMeters)
            .alongWith(Commands.runOnce({ DispenserSubsystem.reverseOuttake = false }))
            .andThen(WristSubsystem.setAngleCommand(ElevatorConstants.l1.wristGoalRotations))    }

    fun l2Safe(): Command {
        return Drivetrain.driveBackCommand().withTimeout(0.2)
            .andThen(ElevatorSubsystem.setHeightSafeCommand(ElevatorConstants.l2.elevatorGoalMeters)
                .alongWith(Commands.runOnce({ DispenserSubsystem.reverseOuttake = true }))
                .andThen(WristSubsystem.setAngleCommand(ElevatorConstants.l2.wristGoalRotations)))
            .andThen(Drivetrain.driveForwardCommand().withTimeout(0.2))
    }

    fun l3Safe(): Command {
        return Drivetrain.driveBackCommand().withTimeout(0.2)
            .andThen(ElevatorSubsystem.setHeightSafeCommand(ElevatorConstants.l3.elevatorGoalMeters)
                .alongWith(Commands.runOnce({ DispenserSubsystem.reverseOuttake = true }))
                .andThen(WristSubsystem.setAngleCommand(ElevatorConstants.l3.wristGoalRotations)))
            .andThen(Drivetrain.driveForwardCommand().withTimeout(0.2))
    }

    fun l4Safe(): Command {
        return Drivetrain.driveBackCommand().withTimeout(0.2)
            .andThen(ElevatorSubsystem.setHeightSafeCommand(ElevatorConstants.l4.elevatorGoalMeters)
            .alongWith(Commands.runOnce({ DispenserSubsystem.reverseOuttake = true }))
            .andThen(WristSubsystem.setAngleCommand(ElevatorConstants.l4.wristGoalRotations)))
            .andThen(Drivetrain.driveForwardCommand().withTimeout(0.2))
    }

    fun stow(): Command {
        return WristSubsystem.setAngleCommand(ElevatorConstants.l4Out.wristGoalRotations)
            .andThen(ElevatorSubsystem.setHeightSafeCommand(ElevatorConstants.stow.elevatorGoalMeters))
            .andThen(WristSubsystem.setAngleCommand(ElevatorConstants.stow.wristGoalRotations))
    }

    fun afterOuttake(): Command {
        return Drivetrain.driveBackCommand().withTimeout(0.4)
            .andThen(this.stow())

    }


    /* Buffered */
    fun l2Instant(): Command {
        return ElevatorSubsystem.setHeightSafeCommand(ElevatorConstants.l2.elevatorGoalMeters)
                .alongWith(Commands.runOnce({ DispenserSubsystem.reverseOuttake = true }))
                .andThen(WristSubsystem.setAngleCommand(ElevatorConstants.l2.wristGoalRotations))
    }

    fun l3Instant(): Command {
        return ElevatorSubsystem.setHeightSafeCommand(ElevatorConstants.l3.elevatorGoalMeters)
                .alongWith(Commands.runOnce({ DispenserSubsystem.reverseOuttake = true }))
                .andThen(WristSubsystem.setAngleCommand(ElevatorConstants.l3.wristGoalRotations))
    }

    fun l4Instant(): Command {
        return ElevatorSubsystem.setHeightSafeCommand(ElevatorConstants.l4.elevatorGoalMeters)
                .alongWith(Commands.runOnce({ DispenserSubsystem.reverseOuttake = true }))
                .andThen(WristSubsystem.setAngleCommand(ElevatorConstants.l4.wristGoalRotations))
    }

    private var bufferedLevel = ManipulatorPositions.STOW

    fun changeBufferCommand(newBuffer: ManipulatorPositions):Command{
        return Commands.runOnce({ bufferedLevel = newBuffer })
    }

    fun activateBufferCommand():Command{
        return SelectCommand(
            mapOf(
                ManipulatorPositions.STOW to Commands.none(),
                ManipulatorPositions.L1 to l1(),
                ManipulatorPositions.L2 to l2Instant(),
                ManipulatorPositions.L3 to l3Instant(),
                ManipulatorPositions.L4 to l4Instant()
            )
        ) { bufferedLevel }
    }
}