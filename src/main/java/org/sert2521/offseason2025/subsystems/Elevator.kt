package org.sert2521.offseason2025.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.motorcontrollers.SmartMotorControllerConfig
import org.sert2521.offseason2025.ElevatorConstants
import yams.math.ExponentialProfilePIDController
import yams.motorcontrollers.local.SparkWrapper
import kotlin.math.E


object Elevator : SubsystemBase() {
    private val leftElevatorMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)

    private val leftMotorConfig = SmartMotorControllerConfig(this)
        .withMechanismCircumference(ElevatorConstants.circumference)
        .withClosedLoopController(ExponentialProfilePIDController(
            ElevatorConstants.P,
            ElevatorConstants.I,
            ElevatorConstants.D,
            ExponentialProfilePIDController.createElevatorConstraints(
                Volts.of(12.0),
                ElevatorConstants.motors,
                ElevatorConstants.weight,
                ElevatorConstants.radius,
                ElevatorConstants.gearing
            )
        ))
        .withSoftLimit(Meters.of(0.0), Meters.of(0.67))
        .withGearing(ElevatorConstants.gearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("ElevatorMotorLeft", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(false)
        .withFeedforward(ElevatorFeedforward(
            ElevatorConstants.S,
            ElevatorConstants.G,
            ElevatorConstants.V
        ))
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)

    private val leftFullMotor = SparkWrapper(leftElevatorMotor, DCMotor.getNEO(1), leftMotorConfig)
}