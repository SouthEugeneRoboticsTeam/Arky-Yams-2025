package org.sert2521.offseason2025.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.offseason2025.ElectronicIDs
import org.sert2521.offseason2025.RobotConstants
import org.sert2521.offseason2025.WristConstants
import yams.gearing.GearBox
import yams.gearing.MechanismGearing
import yams.mechanisms.config.ArmConfig
import yams.mechanisms.config.MechanismPositionConfig
import yams.mechanisms.positional.Arm
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper

object WristSubsystem : SubsystemBase() {
    private val wristMotor = SparkMax(ElectronicIDs.WRIST_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfig = SmartMotorControllerConfig(this)
        .withClosedLoopController(
            WristConstants.P,
            WristConstants.I,
            WristConstants.D
        )
        .withGearing(
            MechanismGearing(
                GearBox.fromReductionStages(
                    3.0,
                    4.0,
                    40.0 / 15.0
                )
            )
        )
        .withExternalEncoder(wristMotor.absoluteEncoder)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Wrist Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withMotorInverted(false)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)

    private val fullMotor = SparkWrapper(wristMotor, DCMotor.getNEO(1), motorConfig)

    private val positionConfig = MechanismPositionConfig()
        .withMaxRobotHeight(RobotConstants.maxHeight)
        .withMaxRobotLength(RobotConstants.maxLength)


    private val fullConfig = ArmConfig(fullMotor)
        .withMOI(WristConstants.moi.`in`(KilogramSquareMeters))
        .withHardLimit(WristConstants.hardMin, WristConstants.hardMax)
        .withTelemetry("Wrist", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withLength(WristConstants.length)
        .withStartingPosition(Rotations.of(-0.27))

    private val arm = Arm(fullConfig)



    override fun periodic() {
        arm.updateTelemetry()
    }

    override fun simulationPeriodic() {
        arm.simIterate()
    }

    fun getMechanism2dForElevator(): MechanismObject2d {
        return arm.mechanismLigament
    }

    fun setAngle(angle: Angle): Command {
        return arm.setAngle(angle)
    }

    fun setAngleWaitUntilThere(angle: Angle): Command {
        return setAngle(angle).andThen(
            Commands.waitUntil {
                MathUtil.isNear(angle.`in`(Rotations), arm.angle.`in`(Rotations), 0.05)
            }
        )
    }

    fun sysId(): Command {
        return arm.sysId(Volts.of(3.0), Volts.of(3.0).per(Second), Seconds.of(30.0))
    }
}