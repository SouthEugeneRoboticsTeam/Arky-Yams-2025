package org.sert2521.offseason2025

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import yams.gearing.GearBox
import yams.gearing.MechanismGearing
import kotlin.math.PI

object RobotConstants {
    val maxHeight = Inches.of(84.0)
    val maxLength = Inches.of(21.4).plus(Inches.of(14.0))


}

object DispenserConstants{
    val moi = KilogramSquareMeters.of(0.0)

    val gearing = MechanismGearing(
        GearBox.fromReductionStages(2.0)
    )
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

    val hardMin = Degrees.of(0.0)
    val hardMax = Degrees.of(0.0)

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
}