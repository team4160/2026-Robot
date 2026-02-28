package frc.robot.systems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GenericConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.field.FieldConstants;
import frc.robot.utils.field.ZoneTrigger;
import java.util.function.Supplier;

/**
 * Superstructure coordinates the shooter, turret, hood, and intake subsystems for unified control
 * during shooting operations.
 */
public class ScoringSystem {

	public final ShooterSubsystem shooter;
	public final TurretSubsystem turret;
	public final HoodSubsystem hood;
	public final SwerveSubsystem swerve;
	public final IntakeSubsystem intake;
	public final IntakeArmSubsystem intakeArm;
	public final KickerSubsystem kicker;
	public final SpindexerSubsystem spindexer;

	// Tolerance for "at setpoint" checks
	private static final AngularVelocity SHOOTER_TOLERANCE = RPM.of(100);
	private static final Angle TURRET_TOLERANCE = Degrees.of(1);
	private static final Angle HOOD_TOLERANCE = Degrees.of(2);

	// Triggers for readiness checks
	private final Trigger isShooterAtSpeed;
	private final Trigger isTurretOnTarget;
	private final Trigger isHoodOnTarget;
	private final Trigger isReadyToShoot;

	private AngularVelocity targetShooterSpeed = RPM.of(0);
	private Angle targetTurretAngle = Degrees.of(0);
	private Angle targetHoodAngle = Degrees.of(0);

	// Default aim point is red hub
	private Translation3d aimPoint = GenericConstants.AimPoints.BLUE_HUB.value;

	public Translation2d aimTarget = FieldConstants.Hub.topCenterPoint.toTranslation2d();

	public ScoringSystem(
		ShooterSubsystem shooter,
		TurretSubsystem turret,
		HoodSubsystem hood,
		SwerveSubsystem swerve,
		IntakeSubsystem intake,
		IntakeArmSubsystem intakeArm,
		KickerSubsystem kicker,
		SpindexerSubsystem spindexer
	) {
		this.shooter = shooter;
		this.turret = turret;
		this.hood = hood;
		this.swerve = swerve;
		this.intake = intake;
		this.intakeArm = intakeArm;
		this.kicker = kicker;
		this.spindexer = spindexer;

		// Create triggers for checking if mechanisms are at their targets
		this.isShooterAtSpeed = new Trigger(
			() -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) < SHOOTER_TOLERANCE.in(RPM)
		);

		this.isTurretOnTarget = new Trigger(
			() ->
				Math.abs(turret.getRawAngle().in(Degrees) - targetTurretAngle.in(Degrees)) <
				TURRET_TOLERANCE.in(Degrees)
		);

		this.isHoodOnTarget = new Trigger(
			() -> Math.abs(hood.getAngle().in(Degrees) - targetHoodAngle.in(Degrees)) < HOOD_TOLERANCE.in(Degrees)
		);

		this.isReadyToShoot = isShooterAtSpeed.and(isTurretOnTarget).and(isHoodOnTarget);
	}

	/** Stops all mechanisms - shooter stops spinning, turret and hood hold position. */
	public Command stopAllCommand() {
		return Commands.parallel(
			shooter.set(0).asProxy(),
			turret.set(0).asProxy(),
			hood.setDutyCycle(0).asProxy()
		).withName("Superstructure.stopAll");
	}

	/**
	 * Aims the superstructure to specific targets - used for auto-targeting.
	 *
	 * @param shooterSpeed Target shooter speed
	 * @param turretAngle Target turret angle
	 * @param hoodAngle Target hood angle
	 */
	public Command aimCommand(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
		return Commands.runOnce(() -> {
			targetShooterSpeed = shooterSpeed;
			targetTurretAngle = turretAngle;
			targetHoodAngle = hoodAngle;
		})
			.andThen(
				Commands.parallel(
					shooter.setVelocity(shooterSpeed).asProxy(),
					turret.setAngle(turretAngle).asProxy(),
					hood.setAngle(hoodAngle).asProxy()
				)
			)
			.withName("Superstructure.aim");
	}

	public void setShooterSetpoints(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
		targetShooterSpeed = shooterSpeed;
		targetTurretAngle = turretAngle;
		targetHoodAngle = hoodAngle;
	}

	/**
	 * Aims the superstructure using suppliers - useful for dynamic targeting.
	 *
	 * @param shooterSpeedSupplier Supplier for target shooter speed
	 * @param turretAngleSupplier Supplier for target turret angle
	 * @param hoodAngleSupplier Supplier for target hood angle
	 */
	public Command aimDynamicCommand(
		Supplier<AngularVelocity> shooterSpeedSupplier,
		Supplier<Angle> turretAngleSupplier,
		Supplier<Angle> hoodAngleSupplier
	) {
		return Commands.parallel(
			shooter.setVelocityDynamic(shooterSpeedSupplier).asProxy(),
			turret.setAngleDynamic(turretAngleSupplier).asProxy(),
			hood.setAngleDynamic(hoodAngleSupplier).asProxy()
		).withName("Superstructure.aimDynamic");
	}

	/** Waits until the superstructure is ready to shoot. */
	public Command waitUntilReadyCommand() {
		return Commands.waitUntil(isReadyToShoot).withName("Superstructure.waitUntilReady");
	}

	/** Aims and waits until ready - combines aim and wait. */
	public Command aimAndWaitCommand(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
		return aimDynamicCommand(() -> shooterSpeed, () -> turretAngle, () -> hoodAngle)
			.andThen(waitUntilReadyCommand())
			.withName("Superstructure.aimAndWait");
	}

	/**
	 * Command that converts the inputed value into RPM and sets the Shooter's velocity in the
	 * forwards direction
	 *
	 * @param velocity
	 * @return A command that runs the Shooter forwards with RPM
	 */
	public Command setShooterRPMForwards(int velocity) {
		return shooter.setVelocity(RPM.of(velocity));
	}

	/**
	 * Command that converts the inputed value into RPM and sets the Shooter's velocity in the reverse
	 * direction
	 *
	 * @param velocity
	 * @return A command that runs the Shooter forwards with RPM
	 */
	public Command setShooterRPMReverse(int velocity) {
		return shooter.setVelocity(RPM.of(-velocity));
	}

	/**
	 * Command that runs the intake and BottomIntake in the forwards direction using DutyCycle
	 *
	 * @param topSpeed
	 * @param bottomSpeed
	 * @return A command that runs the intake and BottomIntake forwards with DutyCycle
	 */
	public Command runIntakeForwards(double speed) {
		return intake.set(speed);
	}

	/**
	 * Command that runs the intake and BottomIntake in the reverse direction using DutyCycle
	 *
	 * @param topSpeed
	 * @param bottomSpeed
	 * @return A command that runs the intake and BottomIntake in reverse with DutyCycle
	 */
	public Command runIntakeReverse(double speed) {
		return intake.set(-speed);
	}

	/**
	 * Command that runs the kicker and Spindexer (after a short delay) in the forwards direction using
	 * DutyCycle
	 *
	 * @param speedKicker
	 * @param speedSpindexer
	 * @return A command that runs the kicker and Spindexer forwards with DutyCycle
	 */
	public Command runKickerAndSpindexerForwards(double speedKicker, double speedSpindexer) {
		return kicker.set(speedKicker).alongWith(new WaitCommand(.25).andThen(spindexer.set(-speedSpindexer)));
	}

	/**
	 * Command that runs the kicker and Spindexer in the reverse direction using DutyCycle
	 *
	 * @param speedKicker
	 * @param speedSpindexer
	 * @return A command that runs the kicker and Spindexer in reverse with DutyCycle
	 */
	public Command runKickerAndSpindexerReverse(double speedKicker, double speedSpindexer) {
		return kicker.set(-speedKicker).alongWith(spindexer.set(speedSpindexer));
	}

	public Command runKickerAndSpindexerAtShooterSpeed() {
		return kicker
			.setVelocity(RPM.of(0.95 * (this.targetShooterSpeed).magnitude()))
			.alongWith(
				new WaitCommand(.25).andThen(
					spindexer.setVelocity((RPM.of(-9 * 0.8 * (this.targetShooterSpeed).magnitude())))
				)
			);
	}

	/**
	 * Command that moves the turret to the right using DutyCycle
	 *
	 * @param dutyCycle
	 * @return A command that turns the Turret rigth with DutyCycle
	 */
	public Command turnTurretRight(double dutyCycle) {
		return turret.set(-dutyCycle);
	}

	/**
	 * Command that moves the turret to the left using DutyCycle
	 *
	 * @param dutyCycle
	 * @return A command that turns the Turret left with DutyCycle
	 */
	public Command turnTurretLeft(double dutyCycle) {
		return turret.set(dutyCycle);
	}

	/**
	 * Command that moves the Arm up using DutyCycle
	 *
	 * @param dutyCycle
	 * @return A command that moves the Arm up with DutyCycle
	 */
	public Command armUp(double dutyCycle) {
		return intakeArm.set(dutyCycle);
	}

	/**
	 * Command that moves the Arm down using DutyCycle
	 *
	 * @param dutyCycle
	 * @return A command that moves the Arm down with DutyCycle
	 */
	public Command armDown(double dutyCycle) {
		return intakeArm.set(-dutyCycle);
	}

	/**
	 * Command that moves the Arm up and down to specific setpoints using position control
	 *
	 * @return A command that moves the Arm up and down to specific setpoints using position control
	 */
	public Command useArmToAgitate() {
		return intakeArm
			.runToAngle(Degrees.of(-10), Degrees.of(1))
			.andThen(intakeArm.runToAngle(Degrees.of(-90), Degrees.of(1)));
	}

	public Command setIntakeUpAndHold(Angle angle) {
		return intakeArm.setAngle(angle);
	}

	public Command setIntakeDownAndHold(Angle angle) {
		return intakeArm.setAngle(angle);
	}

	public Command spindexerCommand(AngularVelocity velocity) {
		return spindexer.setVelocity(velocity);
	}

	// public Command shootWithSpin(
	//     AngularVelocity shootVelocity, AngularVelocity spinVelocity, AngularVelocity kickerVelocity)
	// {
	//   return shootCommand(shootVelocity)
	//       .alongWith(new WaitCommand(.5))
	//       .andThen(kickerAndSpindexer(kickerVelocity, spinVelocity));
	// }

	// TODO this should run the runTo command
	public Command intakeSetAndStart(Angle angle, double topSpeed, double bottomSpeed) {
		return intakeArm.setAngle(angle).andThen(intake.set(topSpeed));
	}

	public Command shooterAndKicker(AngularVelocity shooterVelocity, AngularVelocity kickerVelocity) {
		return shooter
			.setVelocity(shooterVelocity)
			.alongWith(new WaitCommand(.5).andThen(kicker.setVelocity(kickerVelocity)));
	}

	public Command setTurretForward() {
		return turret.setAngle(Degrees.of(0)).withName("Superstructure.setTurretForward");
	}

	public Command setTurretLeft() {
		return turret.setAngle(Degrees.of(45)).withName("Superstructure.setTurretLeft");
	}

	public Command setTurretRight() {
		return turret.setAngle(Degrees.of(-45)).withName("Superstructure.setTurretRight");
	}

	// Getters for current state
	public AngularVelocity getShooterSpeed() {
		return shooter.getSpeed();
	}

	public Angle getTurretAngle() {
		return turret.getRawAngle();
	}

	public Angle getHoodAngle() {
		return hood.getAngle();
	}

	public AngularVelocity getTargetShooterSpeed() {
		return targetShooterSpeed;
	}

	public Angle getTargetTurretAngle() {
		return targetTurretAngle;
	}

	public Angle getTargetHoodAngle() {
		return targetHoodAngle;
	}

	public Translation3d getAimPoint() {
		return aimPoint;
	}

	public void setAimPoint(Translation3d newAimPoint) {
		this.aimPoint = newAimPoint;
	}

	public Rotation3d getAimRotation3d() {
		// See
		// https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
		return new Rotation3d(
			Degrees.of(0), // no roll 🤞
			Degrees.of(0), // hood.getAngle().unaryMinus(), // pitch is negative hood angle
			turret.getRawAngle()
		);
	}

	public Pose3d getShooterPose() {
		// Position of the shooter relative to the "front" of the robot. Rotation
		// element is based on hood and turret angles
		return new Pose3d(
			new Translation3d(Inches.of(-5.25).in(Meters), Inches.of(5.25).in(Meters), Inches.of(16.945).in(Meters)),
			// Inches.of(-5.25).in(Meters), Inches.of(5.25).in(Meters), Inches.of(16.945).in(Meters)),
			// real robot values
			getAimRotation3d()
		);
	}

	/** A class that holds various triggers for control logic. */
	public static class CustomTriggers {

		public static ZoneTrigger bumpZone = new ZoneTrigger(
			"Bump",
			Pair.of(new Translation2d(3.75, 1.5), new Translation2d(5.5, 3.5)),
			Pair.of(new Translation2d(3.75, 4.5), new Translation2d(5.5, 6.5)),
			Pair.of(new Translation2d(11, 4.5), new Translation2d(12.75, 6.5)),
			Pair.of(new Translation2d(11, 1.5), new Translation2d(12.75, 3.5))
		);

		public static ZoneTrigger scoringZone = new ZoneTrigger(
			"Scoring",
			Pair.of(new Translation2d(1.5, 0.5), new Translation2d(3.5, 7.5))
		);

		public static ZoneTrigger leftNeutralZone = new ZoneTrigger(
			"Left Neutral",
			Pair.of(
				new Translation2d(FieldConstants.LinesVertical.starting, FieldConstants.LinesHorizontal.center),
				new Translation2d(FieldConstants.LinesVertical.oppAllianceZone, FieldConstants.fieldWidth)
			)
		);

		public static ZoneTrigger rightNeutralZone = new ZoneTrigger(
			"Right Neutral",
			Pair.of(
				new Translation2d(FieldConstants.LinesVertical.starting, 0),
				new Translation2d(FieldConstants.LinesVertical.oppAllianceZone, FieldConstants.LinesHorizontal.center)
			)
		);
	}
}
