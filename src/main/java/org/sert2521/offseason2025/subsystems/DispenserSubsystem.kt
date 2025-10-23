package org.sert2521.offseason2025.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.sert2521.offseason2025.DispenserConstants
import org.sert2521.offseason2025.ElectronicIDs
import org.sert2521.offseason2025.ElectronicIDs.DISPENSER_BEAMBREAK_ID
import org.sert2521.offseason2025.ElectronicIDs.RAMP_BEAMBREAK_ID
import yams.mechanisms.config.SensorConfig
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper

object DispenserSubsystem: SubsystemBase() {
    private val dispenserMotor = SparkMax(ElectronicIDs.DISPENSER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfig = SmartMotorControllerConfig(this)
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Dispenser Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withGearing(DispenserConstants.dispenserGearing)
        .withMomentOfInertia(DispenserConstants.dispenserMoi.`in`(KilogramSquareMeters))

    private val fullMotor = SparkWrapper(dispenserMotor, DCMotor.getNEO(1), motorConfig)

    private val beambreakRamp = DigitalInput(RAMP_BEAMBREAK_ID)
    private val beambreakDispenser = DigitalInput(DISPENSER_BEAMBREAK_ID)

    private val coralSensors = SensorConfig("Coral Beambreaks")
        .withField("Ramp Beambreak", beambreakRamp::get, false)
        .withField("Dispenser Beambreak", beambreakDispenser::get, false)
        .sensor

    override fun periodic() {
        fullMotor.updateTelemetry()
    }

    override fun simulationPeriodic() {
        fullMotor.simIterate()
    }

    /* Functions */

    fun getRampBlocked():Boolean{
        return !coralSensors.getAsBoolean("Ramp Beambreak")
    }

    fun getDispenserBlocked():Boolean{
        return !coralSensors.getAsBoolean("Dispenser Beambreak")
    }

    fun getBlocked():Boolean{
        return getRampBlocked() || getDispenserBlocked()
    }

    fun setSpeed(speed : Double){
        fullMotor.dutyCycle = speed
    }

    /* Commands */




}