package org.sert2521.offseason2025

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import org.sert2521.offseason2025.utils.ManipulatorGoalState
import yams.gearing.GearBox
import yams.gearing.MechanismGearing
import kotlin.math.PI

object ManipulatorSetpoints {
    val intake = ManipulatorGoalState(Meters.of(0.015), Rotations.of(-0.27))
    val l1 = ManipulatorGoalState(Meters.of(0.015), Rotations.of(-0.182))
    val l2 = ManipulatorGoalState(Meters.of(0.08), Rotations.of(0.22))
    val l3 = ManipulatorGoalState(Meters.of(0.28), Rotations.of(0.22))
    val l4 = ManipulatorGoalState(Meters.of(0.67 - 0.01), Rotations.of(0.124))

    val l4Out = ManipulatorGoalState(Meters.of(0.015), Rotations.of(0.254))
    val stow = ManipulatorGoalState(Meters.of(0.015), Rotations.of(0.21))
}

object RobotConstants {
    val maxHeight = Inches.of(84.0)
    val maxLength = Inches.of(21.4).plus(Inches.of(14.0))
}

object DispenserConstants{
    val dispenserMoi = KilogramSquareMeters.of(0.0)

    val dispenserGearing = MechanismGearing(
        GearBox.fromReductionStages(2.0)
    )

    const val INTAKE_SPEED_FIRST = 0.2
    const val INTAKE_SPEED_SECOND = -0.1
    const val INTAKE_SPEED_THIRD = 0.06

    const val OUTTAKE_NORMAL_SPEED = 0.3
    const val OUTTAKE_L4_SPEED = -0.2
    const val OUTTAKE_TIME = 1.0

    const val DISPENSER_P = 0.0
    const val DISPENSER_D = 0.0
}

object ElevatorConstants{
    private val chainPitch = Inches.of(0.25)
    private const val TOOTH_COUNT = 22
    val circumference = chainPitch.times(TOOTH_COUNT.toDouble())
    val radius = circumference.div(2 * PI)

    val weight = Pounds.of(16.0)
    val motors = DCMotor.getNEO(2)

    val gearing = MechanismGearing(GearBox.fromReductionStages(3.0, 4.0))

    const val P = 0.0
    const val I = 0.0
    const val D = 0.0

    const val S = 0.0
    const val G = 0.0
    const val V = 0.0
}

object WristConstants{
    const val P = 0.0
    const val I = 0.0
    const val D = 0.0

    val moi = KilogramSquareMeters.of(0.0)

    val length = Inches.of(10.8)

    val hardMin = Rotations.of(-0.27)
    val hardMax = Rotations.of(0.254)

    val gearing = MechanismGearing(
        GearBox.fromReductionStages(
            3.0,
            4.0,
            40.0 / 15.0
        )
    )

    val elevatorToWrist = MechanismLigament2d("Connector", 434.892930783, 66.56)
}

object ElectronicIDs{
    const val ELEVATOR_LEFT_MOTOR_ID = 13
    const val ELEVATOR_RIGHT_MOTOR_ID = 14

    const val WRIST_MOTOR_ID = 5

    const val DISPENSER_MOTOR_ID = 17

    const val DISPENSER_BEAMBREAK_ID = 8
    const val RAMP_BEAMBREAK_ID = 9

    const val RAMP_MOTOR_ID = 18
}

object RampConstants {
    val rampMoi = KilogramSquareMeters.of(0.0)

    val rampGearing = MechanismGearing(
        GearBox.fromReductionStages(6.0)
    )
}

object ExampleConstants {
    const val EXAMPLE_ID = 1

    val exampleGearing = MechanismGearing(
        GearBox.fromReductionStages(3.0, 4.0, 2.5, 0.2)
    )

    val moi = 0.2
}