package org.sert2521.offseason2025.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.offseason2025.ElectronicIDs
import org.sert2521.offseason2025.WristConstants
import yams.gearing.GearBox
import yams.gearing.MechanismGearing
import yams.mechanisms.positional.Arm
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper

object DispenserSubsystem: SubsystemBase() {
    private val dispenserMotor = SparkMax(ElectronicIDs.DISPENSER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfig = SmartMotorControllerConfig(this)
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Dispenser Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withGearing(WristConstants.gearing)
        .withMomentOfInertia(WristConstants.moi.`in`(KilogramSquareMeters))

    private val fullMotor = SparkWrapper(dispenserMotor, DCMotor.getNEO(1), motorConfig)

    override fun periodic() {
        fullMotor.updateTelemetry()
    }

    override fun simulationPeriodic() {
        fullMotor.simIterate()
    }

    fun set(speed : Double){
        fullMotor.dutyCycle = speed
    }

    fun setSpeedCommand(speed: Double):Command{
        return run { set(speed) }
    }
}