// package frc.robot.subsystems;
// import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import yams.mechanisms.positional.Arm;
// public class IntakeArmSubsystem extends SubsystemBase {
// 	private TalonFX armLeader = new TalonFX(11);
// 	private TalonFX armFollower = new TalonFX(12);
// 	final CommandXboxController operatorXbox = new CommandXboxController(1);
// 	// private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
// 	// 	.withControlMode(ControlMode.CLOSED_LOOP)
// 	// 	// Feedback Constants (PID Constants)
// 	// 	.withClosedLoopController(1, 0, 0) // , DegreesPerSecond.of(30), DegreesPerSecondPerSecond.of(45))
// 	// 	.withSimClosedLoopController(35, 0, 0) // , DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
// 	// 	// Feedforward Constants
// 	// 	.withFeedforward(new ArmFeedforward(0, 0, 0))
// 	// 	.withSimFeedforward(new ArmFeedforward(0, 0, 0))
// 	// 	// Telemetry name and verbosity level
// 	// 	.withTelemetry("Arm Motor", GenericConstants.kTelemetryVerbosity)
// 	// 	// Gearing from the motor rotor to final shaft.
// 	// 	// In this example GearBox.fromReductionStages(3,4) is the same as
// 	// 	// GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
// 	// 	// You could also use .withGearing(12) which does the same thing.
// 	// 	.withGearing(new MechanismGearing(GearBox.fromStages("38:10")))
// 	// 	// Motor properties to prevent over currenting.
// 	// 	.withMotorInverted(false)
// 	// 	.withIdleMode(MotorMode.BRAKE)
// 	// 	.withStatorCurrentLimit(Amps.of(-1))
// 	// 	.withClosedLoopRampRate(Seconds.of(0.25))
// 	// 	.withOpenLoopRampRate(Seconds.of(0.25))
// 	// 	.withFollowers(Pair.of(armFollower, true));
// 	// private SmartMotorController SmartMotorController = new TalonFXWrapper(
// 	// 	armLeader,
// 	// 	DCMotor.getFalcon500(2),
// 	// 	smcConfig
// 	// );
// 	// private ArmConfig armCfg = new ArmConfig(SmartMotorController)
// 	// 	// Soft limit is applied to the SmartMotorControllers PID
// 	// 	// .withSoftLimits(Degrees.of(-100), Degrees.of(0))
// 	// 	// Hard limit is applied to the simulation.
// 	// 	// .withHardLimit(Degrees.of(-110), Degrees.of(10))
// 	// 	// Starting position is where your arm starts
// 	// 	.withStartingPosition(Degrees.of(0))
// 	// 	// Length and mass of your arm for sim.
// 	// 	.withLength(Feet.of(3))
// 	// 	.withMass(Pounds.of(1))
// 	// 	// Telemetry name and verbosity for the arm.
// 	// 	.withTelemetry("Arm", GenericConstants.kTelemetryVerbosity);
// 	// private Arm arm = new Arm(armCfg);
// 	/**
// 	 * Set the angle of the arm.
// 	 *
// 	 * @param angle Angle to go to.
// 	 */
// 	// public Command setAngle(Angle angle) {
// 	// 	return arm.setAngle(angle);
// 	// }
// 	/**
// 	 * Move the arm to the desired Angle with a given tolerance, then end the command when within
// 	 * tolerance.
// 	 *
// 	 * @param angle
// 	 * @param tolerance
// 	 * @return A command that will move the Arm to the desired Angle within the desired tolerance,
// 	 *     then end the command
// 	 */
// 	// public Command runToAngle(Angle angle, Angle tolerance) {
// 	// 	return arm.runTo(angle, tolerance);
// 	// }
// 	/**
// 	 * Move the arm up and down.
// 	 *
// 	 * @param dutycycle [-1, 1] speed to set the arm too.
// 	 */
// 	public void set(double dutycycle) {
// 		// return arm.set(dutycycle);
// 		armLeader.set(dutycycle);
// 		armFollower.set(dutycycle);
// 	}
// 	/** Run sysId on the {@link Arm} */
// 	// public Command sysId() {
// 	// 	return arm.sysId(Volts.of(5), Volts.of(1).per(Second), Seconds.of(8));
// 	// }
// 	public IntakeArmSubsystem() {}
// 	/**
// 	 * Example command factory method.
// 	 *
// 	 * @return a command
// 	 */
// 	// public Command exampleMethodCommand() {
// 	// 	// Inline construction of command goes here.
// 	// 	// Subsystem::RunOnce implicitly requires `this` subsystem.
// 	// 	return runOnce(() -> {
// 	// 		/* one-time action goes here */
// 	// 	});
// 	// }
// 	/**
// 	 * An example method querying a boolean state of the subsystem (for example, a digital sensor).
// 	 *
// 	 * @return value of some boolean subsystem state, such as a digital sensor.
// 	 */
// 	// public boolean exampleCondition() {
// 	// 	// Query some boolean state, such as a digital sensor.
// 	// 	return false;
// 	// }
// 	@Override
// 	public void periodic() {
// 		// This method will be called once per scheduler run
// 		// arm.updateTelemetry();
// 		if (operatorXbox.x().getAsBoolean()) set(1);
// 		else set(0);
// 	}
// 	// @Override
// 	// public void simulationPeriodic() {
// 	// 	// This method will be called once per scheduler run during simulation
// 	// 	arm.simIterate();
// 	// }
// }
