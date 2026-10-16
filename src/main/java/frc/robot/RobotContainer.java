// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// End of San Diego

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.ShotingOnTheFlyConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
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

	// final ScoringSystem scoringSystem = new ScoringSystem(
	// 	shooter,
	// 	turret,
	// 	hood,
	// 	drivebase,
	// 	intake,
	// 	intakeArm,
	// 	kicker,
	// 	spindexer
	// );

	/**
	 * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
	 * velocity.
	 */
	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
		drivebase.getSwerveDrive(),
		() -> EqualsUtil.sensitivity(-driverXbox.getLeftY(), 0.75) * driveTranslationScale(),
		() -> EqualsUtil.sensitivity(-driverXbox.getLeftX(), 0.75) * driveTranslationScale()
	)
		.withControllerRotationAxis(() -> EqualsUtil.sensitivity(-driverXbox.getRightX(), 0.75) * driveRotationScale())
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
		() -> driverXbox.getLeftY() * driveTranslationScale(),
		() -> driverXbox.getLeftX() * driveTranslationScale()
	)
		.withControllerRotationAxis(() -> driverXbox.getRawAxis(2) * driveRotationScale())
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

	// Teleop shoot-on-the-move command, created in configureBindings. Read by the drive scaling below.
	private ShootOnTheMoveCommand shootOnTheMove;

	/** True while SOTM is tracking and the operator is feeding, if slow drive is enabled on the dashboard. */
	private boolean sotmSlowDriveActive() {
		boolean active =
			SmartDashboard.getBoolean("SOTM/SlowDriveEnabled", ShotingOnTheFlyConstants.kSlowDriveEnabled) &&
			shootOnTheMove != null &&
			shootOnTheMove.isScheduled() &&
			operatorXbox.getRightTriggerAxis() > 0.5;
		SmartDashboard.putBoolean("SOTM/SlowDriveActive", active);
		return active;
	}

	private boolean trainingModeOn() {
		return SmartDashboard.getBoolean("Drive/TrainingMode", OperatorConstants.kTrainingModeDefault);
	}

	/** Driver speed multiplier: SOTM slow drive and training mode stack (multiply). */
	private double driveTranslationScale() {
		double scale = sotmSlowDriveActive() ? ShotingOnTheFlyConstants.kSlowDriveTranslationScale : 1.0;
		return trainingModeOn() ? scale * OperatorConstants.kTrainingTranslationScale : scale;
	}

	private double driveRotationScale() {
		double scale = sotmSlowDriveActive() ? ShotingOnTheFlyConstants.kSlowDriveRotationScale : 1.0;
		return trainingModeOn() ? scale * OperatorConstants.kTrainingRotationScale : scale;
	}

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// com.revrobotics.
		// com.studica.frc.Navx gyro = (com.studica.frc.Navx) drivebase.getSwerveDrive().getGyro().getIMU();

		// Configure the trigger bindings
		configureBindings();
		RegisterAutos();
		DriverStation.silenceJoystickConnectionWarning(true);
		SmartDashboard.setDefaultBoolean("SOTM/SlowDriveEnabled", ShotingOnTheFlyConstants.kSlowDriveEnabled);
		SmartDashboard.setDefaultBoolean("Drive/TrainingMode", OperatorConstants.kTrainingModeDefault);
		DataLogManager.start();

		// Create the NamedCommands that will be used in PathPlanner
		NamedCommands.registerCommand("test", Commands.print("I EXIST"));

		// Have the autoChooser pull in all PathPlanner autos as options
		autoChooser = AutoBuilder.buildAutoChooser();

		// Add a simple auto option to have the robot drive forward for 1 second then stop
		autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
		autoChooser.addOption("testShoot", NamedCommands.getCommand("shoot"));

		// Put the autoChooser on the SmartDashboard
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	private void RegisterAutos() {
		NamedCommands.registerCommand(
			"runIntake",
			intakeArm.setAngle(Degrees.of(0)).alongWith(intake.set(-0.75).withTimeout(15))
		);
		NamedCommands.registerCommand("stopIntake", intake.set(0).withTimeout(1));
		NamedCommands.registerCommand("raiseIntake", intakeArm.setAngle(Degrees.of(30)).withTimeout(3));

		NamedCommands.registerCommand(
			"shoot",
			shooter
				.setVelocity(RPM.of(4000))
				.withTimeout(0.5)
				.andThen(spindexer.set(0.65).alongWith(kicker.set(-1)))
				.withTimeout(5)
				.finallyDo(end -> shooter.set(0).alongWith(spindexer.set(0).alongWith(kicker.set(0))))
		);

		// Aim only (put in parallel with a path; ends when the path/group ends)
		NamedCommands.registerCommand("aimOnMove", new ShootOnTheMoveCommand(drivebase, turret, hood, shooter));

		// Aim while moving, feed once ready (or after 1.5s), feed for 2s, then stop.
		// The feed sequence is the deadline; SOTM by itself never ends.
		ShootOnTheMoveCommand autoSotm = new ShootOnTheMoveCommand(drivebase, turret, hood, shooter);
		NamedCommands.registerCommand(
			"shootOnMove",
			Commands.deadline(
				Commands.waitUntil(autoSotm::isReady)
					.withTimeout(1.5)
					.andThen(spindexer.set(0.65).alongWith(kicker.set(-1)).withTimeout(2)),
				autoSotm
			)
		);

		// Intake arm down + roller on. Never ends: use in a deadline/race with a path.
		NamedCommands.registerCommand(
			"intakeDownRun",
			intakeArm
				.setAngle(Degrees.of(AutoConstants.kArmDownDeg))
				.alongWith(intake.set(AutoConstants.kIntakeDutyCycle))
		);

		// Intake arm up. Not used in autos yet: the arm can't lift right now. Ends when up (or after a timeout).
		NamedCommands.registerCommand(
			"intakeUp",
			intakeArm
				.runToAngle(Degrees.of(AutoConstants.kArmUpDeg), Degrees.of(AutoConstants.kArmToleranceDeg))
				.withTimeout(AutoConstants.kArmMoveTimeoutSecs)
		);

		NamedCommands.registerCommand("jiggle", jiggle());

		// Stopped shot: aim with SOTM, wait until ready (or timeout), then feed + intake roller + jiggle.
		NamedCommands.registerCommand("autoShoot", autoShoot(AutoConstants.kFeedSecs));
		NamedCommands.registerCommand("autoShootLong", autoShoot(AutoConstants.kLongFeedSecs));

		// Depot: strafe side to side while pressing in, intake down and running, for kShimmySecs
		NamedCommands.registerCommand(
			"depotShimmy",
			Commands.deadline(
				Commands.waitSeconds(AutoConstants.kShimmySecs),
				shimmy(),
				intakeArm.setAngle(Degrees.of(AutoConstants.kArmDownDeg)),
				intake.set(AutoConstants.kIntakeDutyCycle)
			)
		);
	}

	/** Drive forward/back a little (robot relative) so balls settle into the spindexer. Never ends. */
	private Command jiggle() {
		return Commands.repeatingSequence(
			Commands.run(
				() -> drivebase.drive(new ChassisSpeeds(AutoConstants.kJiggleSpeedMps, 0, 0)),
				drivebase
			).withTimeout(AutoConstants.kJiggleHalfPeriodSecs),
			Commands.run(
				() -> drivebase.drive(new ChassisSpeeds(-AutoConstants.kJiggleSpeedMps, 0, 0)),
				drivebase
			).withTimeout(AutoConstants.kJiggleHalfPeriodSecs)
		).finallyDo(() -> drivebase.drive(new ChassisSpeeds()));
	}

	/** Strafe left/right (robot relative) while pushing forward a little. Never ends. */
	private Command shimmy() {
		double push = AutoConstants.kShimmyPushMps;
		double side = AutoConstants.kShimmySpeedMps;
		return Commands.repeatingSequence(
			Commands.run(() -> drivebase.drive(new ChassisSpeeds(push, side, 0)), drivebase).withTimeout(
				AutoConstants.kShimmyHalfPeriodSecs
			),
			Commands.run(() -> drivebase.drive(new ChassisSpeeds(push, -side, 0)), drivebase).withTimeout(
				AutoConstants.kShimmyHalfPeriodSecs
			)
		).finallyDo(() -> drivebase.drive(new ChassisSpeeds()));
	}

	/** Shoot while stopped in auto. Ends after feedSecs of feeding. */
	private Command autoShoot(double feedSecs) {
		ShootOnTheMoveCommand aim = new ShootOnTheMoveCommand(drivebase, turret, hood, shooter);
		Command feed = Commands.parallel(
			spindexer.set(AutoConstants.kSpindexerFeedDutyCycle),
			kicker.set(AutoConstants.kKickerFeedDutyCycle),
			intake.set(AutoConstants.kIntakeDutyCycle), // keep pushing balls up into the spindexer
			jiggle()
		).withTimeout(feedSecs);
		return Commands.deadline(
			Commands.waitUntil(aim::isReady).withTimeout(AutoConstants.kReadyTimeoutSecs).andThen(feed),
			aim
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

		intakeArm.setDefaultCommand(intakeArm.set(0));

		shooter.setDefaultCommand(shooter.set(0));

		spindexer.setDefaultCommand(spindexer.set(0));

		kicker.setDefaultCommand(kicker.set(0));

		// turret.setDefaultCommand(turret.setAngle(Degrees.of(0)));
		turret.setDefaultCommand(turret.set(0));

		hood.setDefaultCommand(hood.setDutyCycle(0));

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
		} else if (DriverStation.isTest()) {
			drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

			driverXbox.rightBumper().onTrue(Commands.none());
		} else {
			driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));
			driverXbox.y().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
		}

		// Shoot on the move: A toggles tracking, right trigger (below) feeds. Rumble when ready to fire.
		shootOnTheMove = new ShootOnTheMoveCommand(drivebase, turret, hood, shooter);
		operatorXbox.a().toggleOnTrue(shootOnTheMove);
		shootOnTheMove
			.readyTrigger()
			.whileTrue(
				Commands.startEnd(
					() -> operatorXbox.setRumble(RumbleType.kBothRumble, 0.4),
					() -> operatorXbox.setRumble(RumbleType.kBothRumble, 0)
				)
			);

		// operatorXbox.a().whileTrue(intake.set(IntakeConstants.kIntakeDutyCycle));
		// operatorXbox.leftTrigger().whileTrue(intake.set(-IntakeConstants.kIntakeDutyCycle));
		operatorXbox
			.leftTrigger()
			.whileTrue(intakeArm.setAngle(Degrees.of(92)))
			.onFalse(intakeArm.setAngle(Degrees.of(0)).withTimeout(3));
		driverXbox.leftTrigger().whileTrue(intake.set(-0.75));
		driverXbox.rightTrigger().whileTrue(intake.set(0.75));
		// operatorXbox.y().whileTrue(shooter.setVelocity(RPM.of(5900)));
		// SmartDashboard.putNumber("ShootSpeed", 6900);
		// operatorXbox.y().and(GameData::canShoot).whileTrue(shooter.setVelocity());
		operatorXbox.y().whileTrue(shooter.setVelocity(RPM.of(4000)));

		// operatorXbox.x().whileTrue(intakeArm.set(1)).whileFalse(intakeArm.set(0));

		operatorXbox.rightTrigger().whileTrue(spindexer.set(0.65));
		operatorXbox.rightTrigger().whileTrue(kicker.set(-0.75)).onFalse(kicker.set(0.25).withTimeout(1));

		operatorXbox.leftBumper().toggleOnTrue(hood.setAngle(Degrees.of(23)));
		operatorXbox.rightBumper().toggleOnTrue(hood.setAngle(Degrees.of(30)));

		// operatorXbox.leftBumper().whileTrue(shooter.sysId());

		operatorXbox.pov(270).whileTrue(turret.set(0.15));
		operatorXbox.pov(90).whileTrue(turret.set(-0.15));

		// Flywheel nudge for SOTM table shots: D-pad up = +RPM (shots short), down = -RPM (shots long)
		operatorXbox
			.pov(0)
			.onTrue(
				Commands.runOnce(() -> ShootOnTheMoveCommand.nudgeFlywheel(ShotingOnTheFlyConstants.kFlywheelNudgeRPM))
			);
		operatorXbox
			.pov(180)
			.onTrue(
				Commands.runOnce(() -> ShootOnTheMoveCommand.nudgeFlywheel(-ShotingOnTheFlyConstants.kFlywheelNudgeRPM))
			);
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
