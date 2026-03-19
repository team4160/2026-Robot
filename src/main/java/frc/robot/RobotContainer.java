// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.systems.ScoringSystem;
import frc.robot.utils.EqualsUtil;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	// Controllers and Button Board
	final CommandXboxController driverXbox = new CommandXboxController(0);
	final CommandXboxController operatorXbox = new CommandXboxController(1);

	// The robot's subsystems and commands are defined here...
	private final SwerveSubsystem drivebase = new SwerveSubsystem(
		new File(Filesystem.getDeployDirectory(), "swerve/kraken")
	);

	// Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
	private final SendableChooser<Command> autoChooser;

	private final ShooterSubsystem shooter = new ShooterSubsystem();
	private final TurretSubsystem turret = new TurretSubsystem();
	private final HoodSubsystem hood = new HoodSubsystem();
	private final IntakeSubsystem intake = new IntakeSubsystem();
	private final IntakeArmSubsystem intakeArm = new IntakeArmSubsystem();
	private final KickerSubsystem kicker = new KickerSubsystem();
	private final SpindexerSubsystem spindexer = new SpindexerSubsystem();

	final ScoringSystem scoringSystem = new ScoringSystem(
		shooter,
		turret,
		hood,
		drivebase,
		intake,
		intakeArm,
		kicker,
		spindexer
	);

	/**
	 * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
	 * velocity.
	 */
	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
		drivebase.getSwerveDrive(),
		() -> EqualsUtil.sensitivity(driverXbox.getLeftY(), 0.75),
		() -> EqualsUtil.sensitivity(driverXbox.getLeftX(), 0.75)
	)
		.withControllerRotationAxis(() -> EqualsUtil.sensitivity(-driverXbox.getRightX(), 0.75))
		.deadband(OperatorConstants.DEADBAND)
		.scaleTranslation(0.8)
		.allianceRelativeControl(true);

	/** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
	SwerveInputStream driveDirectAngle = driveAngularVelocity
		.copy()
		.withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
		.headingWhile(true);

	/** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
	SwerveInputStream driveRobotOriented = driveAngularVelocity
		.copy()
		.robotRelative(true)
		.allianceRelativeControl(false);

	SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
		drivebase.getSwerveDrive(),
		() -> driverXbox.getLeftY(),
		() -> driverXbox.getLeftX()
	)
		.withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
		.deadband(OperatorConstants.DEADBAND)
		.scaleTranslation(0.8)
		.allianceRelativeControl(true);
	// Derive the heading axis with math!
	SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard
		.copy()
		.withControllerHeadingAxis(
			() -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
			() -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2)
		)
		.headingWhile(true)
		.translationHeadingOffset(true)
		.translationHeadingOffset(Rotation2d.fromDegrees(0));

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);
		DataLogManager.start();

		// Create the NamedCommands that will be used in PathPlanner
		NamedCommands.registerCommand("test", Commands.print("I EXIST"));

		RegisterAutos();

		// Have the autoChooser pull in all PathPlanner autos as options
		autoChooser = AutoBuilder.buildAutoChooser();
		//Attila: you shouldn't need these, buildAutoChooser should autopopulate all Pathplanner Autos
		// autoChooser.setDefaultOption("Ball Hoard", AutoBuilder.buildAuto("HoardAuto"));
		// autoChooser.addOption("Scatter Fuel", AutoBuilder.buildAuto("AgroSmash"));

		// We
		// Need
		// More
		// Autos

		// Add a simple auto option to have the robot drive forward for 1 second then stop
		autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

		// Put the autoChooser on the SmartDashboard
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	private void RegisterAutos() {
		// these two are all we really need

		NamedCommands.registerCommand(
			"intake",
			intake.set(-IntakeConstants.kIntakeDutyCycle).andThen(intake.set(IntakeConstants.kIntakeDutyCycle))
		);
		NamedCommands.registerCommand(
			"aimOnMove",
			new ShootOnTheMoveCommand(drivebase, scoringSystem)
				.withName("OperatorControls.aimCommand")
				.alongWith(spindexer.set(-.85).alongWith(kicker.set(-0.25)).withTimeout(0.5))
		);
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
	 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
		Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
		Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
		Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
		Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
		Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
		Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

		if (RobotBase.isSimulation()) {
			drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
		} else {
			drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
		}

		intake.setDefaultCommand(intake.set(0));

		intakeArm.setDefaultCommand(intakeArm.setAngle(Degrees.of(30)));

		shooter.setDefaultCommand(shooter.set(0));

		spindexer.setDefaultCommand(spindexer.set(0));

		kicker.setDefaultCommand(kicker.set(0));

		turret.setDefaultCommand(turret.setAngle(Degrees.of(0)));
		// turret.setDefaultCommand(turret.set(0));

		hood.setDefaultCommand(hood.setAngle(Degrees.of(0)));

		if (Robot.isSimulation()) {
			Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
			// drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
			driveDirectAngleKeyboard.driveToPose(
				() -> target,
				new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
				new ProfiledPIDController(
					5,
					0,
					0,
					new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))
				)
			);
			driverXbox
				.start()
				.onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
			driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
			driverXbox
				.button(2)
				.whileTrue(
					Commands.runEnd(
						() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
						() -> driveDirectAngleKeyboard.driveToPoseEnabled(false)
					)
				);
		}
		if (DriverStation.isTest()) {
			drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

			driverXbox.rightBumper().onTrue(Commands.none());
		} else {
			driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));
			driverXbox.y().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
		}

		// when driver holds left trigger:  enable auto-aim, basically
		driverXbox
			.leftTrigger()
			.whileTrue(new ShootOnTheMoveCommand(drivebase, scoringSystem).withName("OperatorControls.aimCommand"));

		// when op points left stick Y-ax up,  take in fuel
		operatorXbox.axisGreaterThan(1, 0.2).whileTrue(intake.set(-IntakeConstants.kIntakeDutyCycle));

		// manual revving  (suboptimal?)
		// Attila: rather use linear velocity over angular velocity like FeetPerSecond or MetersPerSecond
		operatorXbox.a().whileTrue(shooter.setVelocity(RPM.of(4000)));
		operatorXbox.y().whileTrue(shooter.setVelocity(RPM.of(scoringSystem.getTargetShooterSpeed().magnitude())));

		// send the fuel through  (triggers the fuel shooting)
		operatorXbox.rightTrigger().whileTrue(spindexer.set(.85).alongWith(kicker.set(-0.25)));

		// right stick X-ax right
		operatorXbox.axisGreaterThan(4, 0.2).whileTrue(turret.set(.1));
		// right stick X-ax left
		operatorXbox.axisLessThan(4, -0.2).whileTrue(turret.set(-.1));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// Pass in the selected auto from the SmartDashboard as our desired autnomous commmand
		return autoChooser.getSelected();
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}
}
