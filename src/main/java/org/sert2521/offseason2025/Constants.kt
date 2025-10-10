package org.sert2521.offseason2025

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Pounds
import yams.gearing.GearBox
import yams.gearing.MechanismGearing
import kotlin.math.PI

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