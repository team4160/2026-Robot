package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GenericConstants;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HoodSubsystem extends SubsystemBase {

	private final TalonFX hoodMotor = new TalonFX(51);

	private final SmartMotorControllerConfig hoodMotorConfig = new SmartMotorControllerConfig(this)
		.withGearing(new MechanismGearing(GearBox.fromStages("174:12")))
		.withIdleMode(MotorMode.COAST)
		.withTelemetry("HoodMotor", GenericConstants.kTelemetryVerbosity)
		.withStatorCurrentLimit(Amps.of(40))
		.withMotorInverted(true)
		.withClosedLoopRampRate(Seconds.of(0.25))
		.withOpenLoopRampRate(Seconds.of(0.25))
		.withClosedLoopController(88, 15, 0, RPM.of(200), RotationsPerSecondPerSecond.of(200))
		.withFeedforward(new ArmFeedforward(0, 0.3333, 0))
		.withControlMode(ControlMode.CLOSED_LOOP);

	private final SmartMotorController hoodSMC = new TalonFXWrapper(
		hoodMotor,
		DCMotor.getKrakenX44(1),
		hoodMotorConfig
	);

	private final ArmConfig hoodConfig = new ArmConfig(hoodSMC)
		.withTelemetry("HoodMech", GenericConstants.kTelemetryVerbosity)
		.withLength(Inch.of(9))
		.withMass(Pound.of(1))
		.withStartingPosition(Degrees.of(0))
		.withSoftLimits(Degrees.of(1), Degrees.of(35))
		.withHardLimit(Degrees.of(0), Degrees.of(37)); // The Hood can be modeled as an arm since it has a gravitational force acted upon based on the angle its in

	private final Arm hood = new Arm(hoodConfig);

	public HoodSubsystem() {}

	public Command setAngle(Angle angle) {
		return hood.setAngle(angle);
	}

	public void setAngleDirect(Angle angle) {
		hoodSMC.setPosition(angle);
	}

	public Command setAngleDynamic(Supplier<Angle> angleSupplier) {
		return hood.setAngle(angleSupplier);
	}

	public Angle getAngle() {
		return hood.getAngle();
	}

	public Command sysId() {
		return hood.sysId(
			Volts.of(4.0), // maximumVoltage
			Volts.per(Second).of(0.5), // step
			Seconds.of(8.0) // duration
		);
	}

	public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
		return hood.set(dutyCycleSupplier);
	}

	public Command setDutyCycle(double dutyCycle) {
		return hood.set(dutyCycle);
	}

	@Override
	public void periodic() {
		hood.updateTelemetry();
	}

	@Override
	public void simulationPeriodic() {
		hood.simIterate();
	}
}
