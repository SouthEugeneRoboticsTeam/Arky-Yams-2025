package org.sert2521.offseason2025

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.sert2521.offseason2025.subsystems.ElevatorSubsystem

object Input {
    private val driverController = CommandXboxController(0)
    private val gunnerController = CommandJoystick(1)

    private val maniStow = gunnerController.button(3)
    private val maniL1 = gunnerController.button(8)
    private val maniL2 = gunnerController.button(7)
    private val maniL3 = gunnerController.button(6)
    private val maniL4 = gunnerController.button(5)


    init {
        maniStow.onTrue(ElevatorSubsystem.setHeightSafeCommand(ManipulatorSetpoints.stow.elevatorGoal))
        maniL1.onTrue(ElevatorSubsystem.setHeightSafeCommand(ManipulatorSetpoints.l1.elevatorGoal))
        maniL2.onTrue(ElevatorSubsystem.setHeightSafeCommand(ManipulatorSetpoints.l2.elevatorGoal))
        maniL3.onTrue(ElevatorSubsystem.setHeightSafeCommand(ManipulatorSetpoints.l3.elevatorGoal))
        maniL4.onTrue(ElevatorSubsystem.setHeightSafeCommand(ManipulatorSetpoints.l4.elevatorGoal))
    }
}