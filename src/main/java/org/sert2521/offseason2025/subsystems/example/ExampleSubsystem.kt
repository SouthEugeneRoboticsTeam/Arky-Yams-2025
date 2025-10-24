package org.sert2521.offseason2025.subsystems.example

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.offseason2025.ExampleConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper


/**
 * This subsystem is for explaining what all the settings do, and what to use for each value.
 * It explores all required parts and some non-required parts of the bare minimum subsystem.
 */
object ExampleSubsystem : SubsystemBase(){
    /* Electrical and Config Section */

    /**
     *  [exampleMotor] here is initializing the actual real electrical connection to the motor.
     *  This lets the roboRIO communicate with the Spark Max of that ID.
     *
     *      ... = SparkMax(id, motorType)
     *
     *  @param [id] (Integer): The CAN id that the spark max is on. Electrical can tell you this value.
     *
     *  @param [motorType] (kBrushless/kBrushed): Type of motor, brushed vs brushless. All our motors are kBrushless.
     */
    private val exampleMotor = SparkMax(ExampleConstants.EXAMPLE_ID, SparkLowLevel.MotorType.kBrushless)


    /**
     *  [exampleMotorConfig] here refers to the settings you're putting onto the motor controller.
     *  These settings, after they're applied, get put into the motor controller itself.
     *
     *      ... = SmartMotorControllerConfig(this)
     *
     *  You can apply a value for a certain setting by putting line that looks like this:
     *  `.settingName(value)`
     *
     *  Examples of certain settings are:
     *
     *      .withMotorInverted(true/false)
     *
     *  -Required.
     *
     *  -Says whether the motor should be inverted.
     *
     *  -True means positive speeds refer to CW.
     *
     *  -False means positive speeds refer to CCW.
     *
     *  -In general choose the direction that makes positive values move pieces toward the outtake.
     *
     *      .withStatorCurrentLimit(Amps.of(double))
     *
     *  -Required.
     *
     *  -Says the maximum current that's allowed to go through the motor.
     *
     *  -Around 30.0-40.0 is a good amount.
     *
     *
     *      .withIdleMode(BRAKE/COAST)
     *
     *  -Required
     *
     *  -Says whether the motor should brake when no power is put into it.
     *
     *  -BRAKE means the motor will brake with no power - good for intakes and indexers.
     *
     *  -COAST means the motor will coast with no power - good for flywheels and outtakes.
     *
     *
     *      .withTelemetry(name:String, HIGH/MEDIUM/LOW)
     *
     *
     *  -Required.
     *
     *  -Tells how the motor reports back information to the driver computer.
     *
     *  -Name should be in the form of "Subsystem Motor",
     *          with more explanation if there's multiple motors.
     *
     *  -For the second option, choose HIGH generally.
     *
     *
     *      .withGearing(MechanismGearing(
     *          Gearbox.fromReductionStages(
     *              stageOne:Double, stageTwo:Double, stageThree:Double, ...etc.
     *          )
     *      ))
     *
     *  -Required.
     *
     *  -Tells the motor what the gearing between it and the mechanism is.
     *
     *  -Each stage should be in the structure of (input rotations)/(output rotations).
     *
     *  -Should be put in the constants file.
     *
     *
     *      .withMomentOfInertia(double)
     *
     *  -NOT required.
     *
     *  -Tells simulation what it should simulate the moment of inertia of the subsystem to be.
     *
     *  -In kgm^2.
     */

    private val exampleMotorConfig = SmartMotorControllerConfig(this)
        .withMotorInverted(false) // Required
        .withStatorCurrentLimit(Amps.of(40.0)) // Required
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE) // Required
        .withTelemetry("Example Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        // ^ Required
        .withGearing(ExampleConstants.exampleGearing) // Required


    /**
     *  Finally we have the motor wrapper. For us this will be [SparkWrapper].
     *  This takes in 3 arguments:
     *
     *      ... = SparkWrapper(controller, motor, config)
     *
     *  @param [controller] put the original motor you made at the top.
     *
     *  @param [motor] put the number of motors on the subsystem in here.
     *      Most of the time it will be one unless you've been told otherwise.
     *
     *  @param [config] the config you made earlier.
     */

    private val fullMotor = SparkWrapper(exampleMotor, DCMotor.getNEO(1), exampleMotorConfig)


    // This makes sure the info gets to the driverstation

    override fun periodic() {
        fullMotor.updateTelemetry()
    }

    override fun simulationPeriodic() {
        fullMotor.simIterate()
    }


    /* Functions Section */


    // This is the functions section. Here you can use a variety of functions to shorten code later.
    // You can also make functions that transfer information from subsystem to subsystem.



    /**
     *  @return The angle of the shaft directly on the motor.
     */
    fun getRotorRotations():Angle{
        return fullMotor.rotorPosition
    }

    /**
     *  @return The angle on the shaft after gearing.
     */
    fun getMechanismRotations():Angle{
        return fullMotor.mechanismPosition
    }

    /**
     *  @return
     */




}