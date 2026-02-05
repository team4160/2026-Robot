// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HopperConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class KickerSubsystem extends SubsystemBase {

	// Vendor motor controller object
	private TalonFX kicker = new TalonFX(HopperConstants.kKicker_ID);

	private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
		.withControlMode(ControlMode.CLOSED_LOOP)
		// Feedback Constants (PID Constants)
		.withClosedLoopController(0, 0, 0)
		.withSimClosedLoopController(0, 0, 0)
		// Feedforward Constants
		.withFeedforward(new SimpleMotorFeedforward(0.0, 0.0, 0))
		.withSimFeedforward(new SimpleMotorFeedforward(0.0, 0.0, 0))
		// Telemetry name and verbosity level
		.withTelemetry("KickerMotor", TelemetryVerbosity.HIGH)
		// Gearing from the motor rotor to final shaft.
		// In this example GearBox.fromReductionStages(3,4) is the same as
		// GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
		// You could also use .withGearing(12) which does the same thing.
		.withGearing(new MechanismGearing(GearBox.fromStages("3:1")))
		// Motor properties to prevent over currenting.
		.withMotorInverted(false)
		.withIdleMode(MotorMode.BRAKE)
		.withStatorCurrentLimit(Amps.of(40));

	// Create our SmartMotorController
	private SmartMotorController motor = new TalonFXWrapper(kicker, DCMotor.getFalcon500(1), smcConfig);

	private final FlyWheelConfig kickerConfig = new FlyWheelConfig(motor)
		// Diameter of the flywheel.
		.withDiameter(Inches.of(4))
		// Mass of the flywheel.
		.withMass(Pounds.of(1))
		// Maximum speed of the kicker.
		.withUpperSoftLimit(RPM.of(6784 * 4))
		// Telemetry name and verbosity for the arm.
		.withTelemetry("kickerMech", TelemetryVerbosity.HIGH);

	// Kicker Mechanism
	private FlyWheel kickerWheel = new FlyWheel(kickerConfig);

	/**
	 * Gets the current velocity of the kicker.
	 *
	 * @return Kicker velocity.
	 */
	public AngularVelocity getVelocity() {
		return kickerWheel.getSpeed();
	}

	/**
	 * Set the kicker velocity.
	 *
	 * @param speed Speed to set.
	 * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
	 */
	public Command setVelocity(AngularVelocity speed) {
		return kickerWheel.setSpeed(speed);
	}

	/**
	 * Set the dutycycle of the kicker.
	 *
	 * @param dutyCycle DutyCycle to set.
	 * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
	 */
	public Command set(double dutyCycle) {
		return kickerWheel.set(dutyCycle);
	}

	/** Creates a new ExampleSubsystem. */
	public KickerSubsystem() {}

	/**
	 * Example command factory method.
	 *
	 * @return a command
	 */
	public Command kickerCommand() {
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
	public boolean kickerCondition() {
		// Query some boolean state, such as a digital sensor.
		return false;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		kickerWheel.updateTelemetry();
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
		kickerWheel.simIterate();
	}
}
