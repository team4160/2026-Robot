package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GenericConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeArmSubsystem extends SubsystemBase {

	private TalonFX armLeader = new TalonFX(11);
	private TalonFX armFollower = new TalonFX(12);
	private final CANcoder cancoder = new CANcoder(11);

	private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
		.withControlMode(ControlMode.CLOSED_LOOP)
		// Feedback Constants (PID Constants)
		.withClosedLoopController(25, 2, 1, DegreesPerSecond.of(150), DegreesPerSecondPerSecond.of(150))
		// .withSimClosedLoopController(35, 0, 0) // , DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
		// Feedforward Constants
		.withFeedforward(new ArmFeedforward(0.1, 0.48, 20))
		// .withSimFeedforward(new ArmFeedforward(0, 0, 0))
		// Telemetry name and verbosity level
		.withTelemetry("Arm Motor", GenericConstants.kTelemetryVerbosity)
		// Gearing from the motor rotor to final shaft.
		// In this example GearBox.fromReductionStages(3,4) is the same as
		// GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
		// You could also use .withGearing(12) which does the same thing.
		.withExternalEncoder(cancoder)
		.withUseExternalFeedbackEncoder(true)
		.withGearing(new MechanismGearing(GearBox.fromStages("38:10", "7:1")))
		// Motor properties to prevent over currenting.
		.withMotorInverted(false)
		.withIdleMode(MotorMode.BRAKE)
		.withStatorCurrentLimit(Amps.of(69))
		.withSupplyCurrentLimit(Amps.of(50))
		// .withClosedLoopRampRate(Seconds.of(0.25))
		// .withOpenLoopRampRate(Seconds.of(0.25))
		.withFollowers(Pair.of(armFollower, true));

	private SmartMotorController SmartMotorController = new TalonFXWrapper(
		armLeader,
		DCMotor.getFalcon500(2),
		smcConfig
	);

	private ArmConfig armCfg = new ArmConfig(SmartMotorController)
		// Soft limit is applied to the SmartMotorControllers PID
		.withSoftLimits(Degrees.of(-1.5), Degrees.of(92))
		// Hard limit is applied to the simulation.
		.withHardLimit(Degrees.of(-2), Degrees.of(94))
		// Starting position is where your arm starts
		// .withStartingPosition(Degrees.of(0))
		// Length and mass of your arm for sim.
		.withLength(Feet.of(3))
		.withMass(Pounds.of(1))
		// Telemetry name and verbosity for the arm.
		.withTelemetry("Arm", GenericConstants.kTelemetryVerbosity);

	private Arm arm = new Arm(armCfg);

	/**
	 * Set the angle of the arm.
	 *
	 * @param angle Angle to go to.
	 */
	public Command setAngle(Angle angle) {
		return arm.setAngle(angle);
	}

	/**
	 * Move the arm to the desired Angle with a given tolerance, then end the command when within
	 * tolerance.
	 *
	 * @param angle
	 * @param tolerance
	 * @return A command that will move the Arm to the desired Angle within the desired tolerance,
	 *     then end the command
	 */
	public Command runToAngle(Angle angle, Angle tolerance) {
		return arm.runTo(angle, tolerance);
	}

	/**
	 * Move the arm up and down.
	 *
	 * @param dutycycle [-1, 1] speed to set the arm too.
	 */
	public Command set(double dutycycle) {
		return arm.set(dutycycle);
	}

	/** Run sysId on the {@link Arm} */
	public Command sysId() {
		return arm.sysId(Volts.of(5), Volts.of(1).per(Second), Seconds.of(8));
	}

	public IntakeArmSubsystem() {
		CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();
		cancoderConfiguration.MagnetSensor.MagnetOffset = -0.238525390625;
		cancoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
		cancoder.getConfigurator().apply(cancoderConfiguration);
		cancoder.getPosition().refresh();
	}

	/**
	 * Example command factory method.
	 *
	 * @return a command
	 */
	public Command exampleMethodCommand() {
		// Inline construction of command goes here.
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(() -> {
			/* one-time action goes here */
		});
	}

	/**
	 * An example method querying a boolean state of the subsystem (for example, a digital sensor).
	 *
	 * @return value of some boolean subsystem state, such as a digital sensor.
	 */
	public boolean exampleCondition() {
		// Query some boolean state, such as a digital sensor.
		return false;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("arm angle", arm.getAngle().in(Degree));
		arm.updateTelemetry();
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
		arm.simIterate();
	}
}
