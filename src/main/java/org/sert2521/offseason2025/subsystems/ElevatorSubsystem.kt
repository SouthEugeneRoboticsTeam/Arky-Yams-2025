package org.sert2521.offseason2025.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Pair
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.offseason2025.ElectronicIDs
import org.sert2521.offseason2025.ElevatorConstants
import org.sert2521.offseason2025.RobotConstants
import org.sert2521.offseason2025.WristConstants
import yams.math.ExponentialProfilePIDController
import yams.mechanisms.config.ElevatorConfig
import yams.mechanisms.config.MechanismPositionConfig
import yams.mechanisms.positional.Elevator
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper


object ElevatorSubsystem : SubsystemBase() {
    private val leftElevatorMotor = SparkMax(ElectronicIDs.ELEVATOR_LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
    private val rightElevatorMotor = SparkMax(ElectronicIDs.ELEVATOR_RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val leftMotorConfig = SmartMotorControllerConfig(this)
        .withMechanismCircumference(ElevatorConstants.circumference)
        .withClosedLoopController(
            ExponentialProfilePIDController(
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
            )
        )
        .withSoftLimit(Meters.of(0.0), Meters.of(0.67))
        .withGearing(ElevatorConstants.gearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Elevator Motor (Left)", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(false)
        .withFeedforward(
            ElevatorFeedforward(
                ElevatorConstants.S,
                ElevatorConstants.G,
                ElevatorConstants.V
            )
        )
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        .withFollowers(Pair(rightElevatorMotor, true))

    private val fullMotors = SparkWrapper(leftElevatorMotor, DCMotor.getNEO(2), leftMotorConfig)

    private val elevatorPosition = MechanismPositionConfig()
        .withMaxRobotHeight(RobotConstants.maxHeight)
        .withMaxRobotLength(RobotConstants.maxLength)
        .withRelativePosition(Translation3d(Meters.of(-0.25), Meters.of(0.0), Meters.of(0.5)))

    private val elevatorConfig = ElevatorConfig(fullMotors)
        .withStartingHeight(Meters.of(0.0))
        .withHardLimits(Meters.of(0.0), Meters.of(0.67))
        .withTelemetry("Elevator", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withMechanismPositionConfig(elevatorPosition)
        .withMass(ElevatorConstants.weight)

    private val elevator = Elevator(elevatorConfig)

    init {
        elevator.mechanismLigament.append(
            WristConstants.elevatorToWrist.append(WristSubsystem.getMechanism2dForElevator())
        )
    }

    override fun periodic() {
        elevator.updateTelemetry()
    }

    override fun simulationPeriodic() {
        elevator.simIterate()
    }

    fun setHeight(height: Distance): Command {
        return elevator.setHeight(height)
    }

    fun setHeightUntilThere(height: Distance): Command {
        return setHeight(height).andThen(
            Commands.waitUntil {
                MathUtil.isNear(
                    height.`in`(Meters),
                    elevator.height.`in`(Meters),
                    0.05
                )
            }
        )
    }

    fun sysId(): Command {
        return elevator.sysId(
            Volts.of(12.0),
            Volts.of(12.0).per(Second),
            Seconds.of(30.0)
        )
    }
}