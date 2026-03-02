package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GenericConstants;
import frc.robot.constants.TurretConstants;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

/** Turret that uses YAMS CRT */
public class TurretSubsystem extends SubsystemBase {

	public static final AngularVelocity threshold = DegreesPerSecond.of(5); // Set a threshold
	public static final Angle toleranceAngle = Degrees.of(0.1); // Set a threshold

	private final TalonFX turretMotor;
	private final SmartMotorControllerConfig motorConfig;
	private final SmartMotorController motor;
	private final MechanismPositionConfig robotToMechanism;
	private final PivotConfig pivotConfig;
	private final Pivot turret;
	private final CANcoder cancoderA;
	private final CANcoder cancoderB;
	private final StatusSignal<Angle> absPosition1Signal;
	private final StatusSignal<Angle> absPosition2Signal;
	private final EasyCRTConfig easyCRTConfig;
	private final EasyCRT easyCrtSolver;

	public TurretSubsystem() {
		turretMotor = new TalonFX(TurretConstants.kTurretMotor_ID);

		cancoderA = new CANcoder(TurretConstants.kTurretEnc1_ID);
		cancoderB = new CANcoder(TurretConstants.kTurretEnc2_ID);

		CANcoderConfiguration cancoderConfigurationA = new CANcoderConfiguration();
		cancoderConfigurationA.MagnetSensor.MagnetOffset = TurretConstants.enc1Offset;
		cancoderA.getConfigurator().apply(cancoderConfigurationA);

		CANcoderConfiguration cancoderConfigurationB = new CANcoderConfiguration();
		cancoderConfigurationB.MagnetSensor.MagnetOffset = TurretConstants.enc2Offset;
		cancoderB.getConfigurator().apply(cancoderConfigurationB);

		absPosition1Signal = cancoderA.getAbsolutePosition();
		absPosition2Signal = cancoderB.getAbsolutePosition();

		motorConfig = new SmartMotorControllerConfig(this)
			.withClosedLoopController(0.4491, 0, 0, DegreesPerSecond.of(45), DegreesPerSecondPerSecond.of(45))
			// .withSimClosedLoopController(130, 0, 3.4, DegreesPerSecond.of(1000), DegreesPerSecondPerSecond.of(1500))
			.withSoftLimit(Degrees.of(-30), Degrees.of(222))
			.withFeedforward(new SimpleMotorFeedforward(0.18, 4.1566, 0.11307))
			.withGearing(new MechanismGearing(GearBox.fromStages("5:1")))
			.withIdleMode(MotorMode.COAST)
			.withTelemetry("TurretMotorV2", GenericConstants.kTelemetryVerbosity)
			.withStatorCurrentLimit(Amps.of(40))
			// .withSupplyCurrentLimit(Amps.of(40))
			.withMotorInverted(false)
			.withControlMode(ControlMode.CLOSED_LOOP);

		motor = new TalonFXWrapper(turretMotor, DCMotor.getKrakenX44(1), motorConfig);

		robotToMechanism = new MechanismPositionConfig()
			.withMaxRobotHeight(Inches.of(9))
			.withMaxRobotLength(Inches.of(12))
			.withRelativePosition(
				new Translation3d(
					Inches.of(-5.3), // back from robot center
					Inches.of(4.4), // centered left/right
					Inches.of(11) // up from the floor reference
				)
			);

		pivotConfig = new PivotConfig(motor)
			.withHardLimit(Degrees.of(-30), Degrees.of(222))
			.withTelemetry("Turret", GenericConstants.kTelemetryVerbosity)
			.withStartingPosition(Degrees.of(0))
			.withMechanismPositionConfig(robotToMechanism)
			.withMOI(Meters.of(0.25), Pounds.of(4));

		turret = new Pivot(pivotConfig);
		easyCRTConfig = buildEasyCrtConfig();
		easyCrtSolver = new EasyCRT(easyCRTConfig);
	}

	/** Build the CRT config */
	private EasyCRTConfig buildEasyCrtConfig() {
		return new EasyCRTConfig(absPosition1Signal::getValue, absPosition2Signal::getValue)
			.withCommonDriveGear(1, 200, 19, 21)
			.withAbsoluteEncoderOffsets(Rotations.of(0), Rotations.of(0))
			.withAbsoluteEncoderInversions(false, false)
			.withMechanismRange(Degrees.of(-30), Degrees.of(222))
			.withMatchTolerance(toleranceAngle)
			.withCrtGearRecommendationConstraints(1.2, 15, 60, 40);
	}

	public Command sysId() {
		return turret.sysId(
			Volts.of(4.0), // maximumVoltage
			Volts.per(Second).of(0.5), // step
			Seconds.of(8.0) // duration
		);
	}

	/**
	 * Set the dutycycle of the turret.
	 *
	 * @param dutyCycle DutyCycle to set.
	 * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
	 */
	public Command set(double dutyCycle) {
		return turret.set(dutyCycle);
	}

	public Command setAngle(Angle angle) {
		return turret.setAngle(angle);
	}

	public Command setAngleDynamic(Supplier<Angle> angle) {
		return turret.setAngle(angle);
	}

	public Angle getRawAngle() {
		return turret.getAngle();
	}

	public Angle getRobotAdjustedAngle() {
		return turret.getAngle().plus(Degrees.of(180));
	}

	public double getRobotRelativeYawRadians() {
		return getRawAngle().in(edu.wpi.first.units.Units.Radians);
	}

	public void periodic() {
		if (motor.getRotorVelocity().compareTo(threshold) < 0) {
			// Only update when the mechanism is moving slowly to ensure accurate readings
			easyCrtSolver
				.getAngleOptional()
				.ifPresent(angle -> {
					// Use the angle for your application
					motor.setEncoderPosition(angle); // Set the motor's encoder position to the calculated angle
				});
		}
		turret.updateTelemetry();
	}

	public void simulationPeriodic() {
		turret.simIterate();
	}
}
