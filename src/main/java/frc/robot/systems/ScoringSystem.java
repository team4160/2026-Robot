package frc.robot.systems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GenericConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.Supplier;

/**
 * Superstructure coordinates the shooter, turret, hood, and intake subsystems for unified control
 * during shooting operations.
 */
public class ScoringSystem {

	public final ShooterSubsystem shooter;
	public final TurretSubsystem turret;
	// public final HoodSubsystem hood;
	public final SwerveSubsystem swerve;

	// Tolerance for "at setpoint" checks
	private static final AngularVelocity SHOOTER_TOLERANCE = RPM.of(100);
	private static final Angle TURRET_TOLERANCE = Degrees.of(1);
	// private static final Angle HOOD_TOLERANCE = Degrees.of(2);

	// Triggers for readiness checks
	private final Trigger isShooterAtSpeed;
	private final Trigger isTurretOnTarget;
	// private final Trigger isHoodOnTarget;
	private final Trigger isReadyToShoot;

	private AngularVelocity targetShooterSpeed = RPM.of(0);
	private Angle targetTurretAngle = Degrees.of(0);
	private Angle targetHoodAngle = Degrees.of(0);

	// Default aim point is red hub
	private Translation3d aimPoint = GenericConstants.AimPoints.BLUE_HUB.value;

	public ScoringSystem(ShooterSubsystem shooter, TurretSubsystem turret, SwerveSubsystem swerve) {
		this.shooter = shooter;
		this.turret = turret;
		this.swerve = swerve;

		// Create triggers for checking if mechanisms are at their targets
		this.isShooterAtSpeed = new Trigger(
			() -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) < SHOOTER_TOLERANCE.in(RPM)
		);

		this.isTurretOnTarget = new Trigger(
			() ->
				Math.abs(turret.getRawAngle().in(Degrees) - targetTurretAngle.in(Degrees)) <
				TURRET_TOLERANCE.in(Degrees)
		);

		// this.isHoodOnTarget = new Trigger(
		//     () -> Math.abs(hood.getAngle().in(Degrees) - targetHoodAngle.in(Degrees)) <
		// HOOD_TOLERANCE.in(Degrees));

		this.isReadyToShoot = isShooterAtSpeed.and(isTurretOnTarget); // .and(isHoodOnTarget);
	}

	/** Stops all mechanisms - shooter stops spinning, turret and hood hold position. */
	public Command stopAllCommand() {
		return Commands.parallel(shooter.set(0).asProxy(), turret.set(0).asProxy());
		// hood.set(0).asProxy()).withName("Superstructure.stopAll");
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
				Commands.parallel(shooter.setVelocity(shooterSpeed).asProxy(), turret.setAngle(turretAngle).asProxy())
			)
			// hood.setAngle(hoodAngle).asProxy()))
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
			turret.setAngleDynamic(turretAngleSupplier).asProxy()
			// hood.setAngleDynamic(hoodAngleSupplier).asProxy()
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
		// return hood.getAngle();
		return (Rotations.of(0));
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

	// public Command useRequirement() {
	//   return runOnce(() -> {
	//   });
	// }

	public Pose3d getShooterPose() {
		// Position of the shooter relative to the "front" of the robot. Rotation
		// element is based on hood and turret angles
		// TODO add actual turret pose
		return new Pose3d(
			new Translation3d(Inches.of(-6.25).in(Meters), Inches.of(6.25).in(Meters), Inches.of(14).in(Meters)),
			getAimRotation3d()
		);
	}
}
