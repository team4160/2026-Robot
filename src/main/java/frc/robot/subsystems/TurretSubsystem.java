package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class TurretSubsystem extends SubsystemBase {

	public static final AngularVelocity threshold = DegreesPerSecond.of(5); // Set a threshold
	public static final Angle toleranceAngle = Degrees.of(0.1); // Set a threshold

	private final TalonFX turretMotor;
	private final SmartMotorControllerConfig motorConfig;
	private final SmartMotorController motor;
	private final MechanismPositionConfig robotToMechanism;
	private final PivotConfig pivotConfig;
	private final Pivot turret;
	private final CANcoder cancoder;

	// private boolean zeroed = false;

	public TurretSubsystem() {
		turretMotor = new TalonFX(TurretConstants.kTurretMotor_ID);

		cancoder = new CANcoder(TurretConstants.kTurretEnc_ID);

		CANcoderConfiguration cancoderConfigurationA = new CANcoderConfiguration();
		cancoderConfigurationA.MagnetSensor.MagnetOffset = TurretConstants.encOffset;
		cancoder.getConfigurator().apply(cancoderConfigurationA);
		cancoder.getPosition().refresh();

		motorConfig = new SmartMotorControllerConfig(this)
			.withSoftLimit(Degrees.of(-30), Degrees.of(222))
			.withClosedLoopController(69, 6.9, 0, DegreesPerSecond.of(5000), DegreesPerSecondPerSecond.of(2000))
			// .withSimClosedLoopController(130, 0, 3.4, DegreesPerSecond.of(1000), DegreesPerSecondPerSecond.of(1500))
			// .withFeedforward(new SimpleMotorFeedforward(0.18, 4.0, 0.11307))
			.withGearing(new MechanismGearing(GearBox.fromStages("5:1", "200:20")))
			.withIdleMode(MotorMode.COAST)
			.withTelemetry("TurretMotorV2", GenericConstants.kTelemetryVerbosity)
			// .withStatorCurrentLimit(Amps.of(40))
			.withSupplyCurrentLimit(Amps.of(40))
			.withMotorInverted(false)
			// .withStartingPosition(cancoder.getPosition().getValue().times(21 / 200))
			// .withStartingPosition(Degrees.of(0))
			.withExternalEncoder(cancoder)
			.withExternalEncoderGearing(200 / 21)
			.withUseExternalFeedbackEncoder(true)
			.withControlMode(ControlMode.CLOSED_LOOP);

		motor = new TalonFXWrapper(turretMotor, DCMotor.getFalcon500(1), motorConfig);

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
			// .withStartingPosition(cancoder.getPosition().getValue().times(21 / 200))
			// .withStartingPosition(Degrees.of(0))
			.withMechanismPositionConfig(robotToMechanism)
			.withMOI(Meters.of(0.25), Pounds.of(4));

		turret = new Pivot(pivotConfig);
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
		// if (!zeroed) {
		// 	cancoder.getAbsolutePosition().refresh();
		// 	if (cancoder.getAbsolutePosition().getValueAsDouble() != 0) {
		// 		zeroed = true;
		// 		// motor.setPosition(cancoder.getPosition().getValue().times(21 / 200));
		// 		turret.setAngle(cancoder.getAbsolutePosition().getValue());
		// 	}
		// }
		// turretMotor.setPosition(cancoder.getAbsolutePosition().getValue());
		// turretMotor.setPosition(3);
		SmartDashboard.putNumber("turret angle", turret.getAngle().in(Degree));
		turret.updateTelemetry();
	}

	public void simulationPeriodic() {
		turret.simIterate();
	}
}