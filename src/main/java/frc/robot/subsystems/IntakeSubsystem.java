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
import frc.robot.constants.IntakeConstants;
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

public class IntakeSubsystem extends SubsystemBase {

	// Vendor motor controller object
	private TalonFX Intake = new TalonFX(IntakeConstants.kIntake_ID);

	private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
		.withControlMode(ControlMode.CLOSED_LOOP)
		// Feedback Constants (PID Constants)
		.withClosedLoopController(0.01, 0.1, 0)
		// .withSimClosedLoopController(1, 0, 0)
		// Feedforward Constants
		.withFeedforward(new SimpleMotorFeedforward(0.02, 0.05, 0))
		// .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
		// Telemetry name and verbosity level
		.withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
		// Gearing from the motor rotor to final shaft.
		// In this example GearBox.fromReductionStages(3,4) is the same as
		// GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
		// You could also use .withGearing(12) which does the same thing.
		.withGearing(new MechanismGearing(GearBox.fromStages("72:24")))
		// Motor properties to prevent over currenting.
		.withMotorInverted(false)
		.withIdleMode(MotorMode.COAST)
		.withStatorCurrentLimit(Amps.of(40));

	// Create our SmartMotorController
	private SmartMotorController IntakeMotor = new TalonFXWrapper(Intake, DCMotor.getKrakenX60(1), smcConfig);

	private final FlyWheelConfig IntakeConfig = new FlyWheelConfig(IntakeMotor)
		// Diameter of the flywheel.
		.withDiameter(Inches.of(4))
		// Mass of the flywheel.
		.withMass(Pounds.of(1))
		// Maximum speed of the shooter.
		.withUpperSoftLimit(RPM.of(1000))
		// Telemetry name and verbosity for the arm.
		.withTelemetry("IntakeWheel", TelemetryVerbosity.HIGH);

	// Shooter Mechanism
	private FlyWheel IntakeWheel = new FlyWheel(IntakeConfig);

	/**
	 * Gets the current velocity of the shooter.
	 *
	 * @return Shooter velocity.
	 */
	public AngularVelocity getVelocity() {
		return IntakeWheel.getSpeed();
	}

	/**
	 * Set the shooter velocity.
	 *
	 * @param speed Speed to set.
	 * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
	 */
	public Command setVelocity(AngularVelocity speed) {
		return IntakeWheel.setSpeed(speed);
	}

	/**
	 * Set the dutycycle of the shooter.
	 *
	 * @param dutyCycle DutyCycle to set.
	 * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
	 */
	public Command set(double dutyCycle) {
		return IntakeWheel.set(dutyCycle);
	}

	/** Creates a new ExampleSubsystem. */
	public IntakeSubsystem() {}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		IntakeWheel.updateTelemetry();
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
		IntakeWheel.simIterate();
	}
}
