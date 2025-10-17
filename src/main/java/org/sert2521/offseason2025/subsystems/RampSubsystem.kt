package org.sert2521.offseason2025.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.offseason2025.DispenserConstants
import org.sert2521.offseason2025.ElectronicIDs
import org.sert2521.offseason2025.ElectronicIDs.DISPENSER_BEAMBREAK_ID
import org.sert2521.offseason2025.ElectronicIDs.RAMP_BEAMBREAK_ID
import org.sert2521.offseason2025.RampConstants
import yams.mechanisms.config.SensorConfig
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper

object RampSubsystem: SubsystemBase() {
    private val rampMotor = SparkMax(ElectronicIDs.RAMP_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfig = SmartMotorControllerConfig(this)
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Ramp Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withGearing(RampConstants.rampGearing)
        .withMomentOfInertia(RampConstants.rampMoi.`in`(KilogramSquareMeters))

    private val fullMotor = SparkWrapper(rampMotor, DCMotor.getNeo550(1), motorConfig)

    override fun periodic() {
        fullMotor.updateTelemetry()
    }

    override fun simulationPeriodic() {
        fullMotor.simIterate()
    }

    fun setSpeed(speed : Double){
        fullMotor.dutyCycle = speed
    }


}