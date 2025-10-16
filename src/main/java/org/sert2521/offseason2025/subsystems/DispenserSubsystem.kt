package org.sert2521.offseason2025.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.offseason2025.DispenserConstants
import org.sert2521.offseason2025.ElectronicIDs
import org.sert2521.offseason2025.ElectronicIDs.DISPENSER_BEAMBREAK_ID
import org.sert2521.offseason2025.ElectronicIDs.RAMP_BEAMBREAK_ID
import org.sert2521.offseason2025.WristConstants
import yams.gearing.GearBox
import yams.gearing.MechanismGearing
import yams.mechanisms.config.SensorConfig
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
        .withGearing(DispenserConstants.dispenserGearing)
        .withMomentOfInertia(DispenserConstants.dispenserMoi.`in`(KilogramSquareMeters))

    private val fullMotor = SparkWrapper(dispenserMotor, DCMotor.getNEO(1), motorConfig)

    private val beambreakRamp = DigitalInput(RAMP_BEAMBREAK_ID)
    private val beambreakDispenser = DigitalInput(DISPENSER_BEAMBREAK_ID)

    private val sensor = SensorConfig("Coral Beambreaks")
        .withField("Ramp Beambreak", beambreakRamp::get, false)
        .withField("Dispenser Beambreak", beambreakDispenser::get, false)
        .sensor

    override fun periodic() {
        fullMotor.updateTelemetry()
        getBlocked()
    }

    override fun simulationPeriodic() {
        fullMotor.simIterate()
    }

    fun getRampBlocked():Boolean{
        return !sensor.getAsBoolean("Ramp Beambreak")
    }

    fun getDispenserBlocked():Boolean{
        return !sensor.getAsBoolean("Dispenser Beambreak")
    }

    fun getBlocked():Boolean{
        return getRampBlocked() || getDispenserBlocked()
    }

    fun setSpeed(speed : Double){
        fullMotor.dutyCycle = speed
    }


}