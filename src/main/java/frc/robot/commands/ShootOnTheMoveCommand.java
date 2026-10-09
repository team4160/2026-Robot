package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ShotingOnTheFlyConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.field.AllianceFlipUtil;
import frc.robot.utils.field.FieldConstants;

/**
 * Continuously aims the turret, hood, and flywheel at the hub while the robot drives.
 *
 * <p>The ball leaves the shooter carrying the turret's field velocity, so we aim from a "virtual" turret position
 * shifted by (turret velocity * time of flight). Time of flight depends on distance, so we iterate to converge.
 *
 * <p>Does not require the drivebase, so the driver keeps full control. Feeding is left to the operator; use
 * {@link #isReady()} to gate it or show it on the dashboard.
 */
public class ShootOnTheMoveCommand extends ParallelCommandGroup {

	private static final InterpolatingDoubleTreeMap flywheelRPMMap = new InterpolatingDoubleTreeMap();
	private static final InterpolatingDoubleTreeMap hoodDegMap = new InterpolatingDoubleTreeMap();
	private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
	private static final double minDistance;
	private static final double maxDistance;

	static {
		double[][] table = ShotingOnTheFlyConstants.kShotTable;
		for (double[] row : table) {
			flywheelRPMMap.put(row[0], row[1]);
			hoodDegMap.put(row[0], row[2]);
			timeOfFlightMap.put(row[0], row[3]);
		}
		minDistance = table[0][0];
		maxDistance = table[table.length - 1][0];

		publishTuningEntries();
	}

	/** Creates the tuning entries if they are missing, without overwriting values typed on the dashboard. */
	private static void publishTuningEntries() {
		SmartDashboard.setDefaultBoolean("SOTM/TuningMode", false);
		SmartDashboard.setDefaultNumber("SOTM/TuningRPM", 4000);
		SmartDashboard.setDefaultNumber("SOTM/TuningHoodDeg", 23);
		SmartDashboard.setDefaultNumber("SOTM/TurretTrimDeg", ShotingOnTheFlyConstants.kTurretTrimDeg);
		SmartDashboard.setDefaultBoolean("SOTM/TurretFlipped", ShotingOnTheFlyConstants.kTurretFlipped);
	}

	private final SwerveSubsystem drivebase;
	private final TurretSubsystem turret;
	private final HoodSubsystem hood;
	private final ShooterSubsystem shooter;

	// Latest solution, read by the mechanism commands every loop
	private double turretSetpointDeg;
	private double hoodSetpointDeg;
	private double flywheelSetpointRPM;
	private boolean inRange = false;
	private boolean turretReachable = false;

	public ShootOnTheMoveCommand(
		SwerveSubsystem drivebase,
		TurretSubsystem turret,
		HoodSubsystem hood,
		ShooterSubsystem shooter
	) {
		this.drivebase = drivebase;
		this.turret = turret;
		this.hood = hood;
		this.shooter = shooter;

		turretSetpointDeg = turret.getRawAngle().in(Degrees);
		hoodSetpointDeg = hood.getAngle().in(Degrees);
		flywheelSetpointRPM = 0;

		// Parallel groups execute children in order, so the solver runs before the mechanisms read it.
		addCommands(
			Commands.run(this::calculate),
			turret.setAngleDynamic(() -> Degrees.of(turretSetpointDeg)),
			hood.setAngleDynamic(() -> Degrees.of(hoodSetpointDeg)),
			shooter.setVelocityDynamic(() -> RPM.of(flywheelSetpointRPM))
		);
		setName("ShootOnTheMove");
	}

	private void calculate() {
		// Predict where the robot will be after the control latency
		ChassisSpeeds robotRelative = drivebase.getRobotVelocity();
		double dt = ShotingOnTheFlyConstants.kPhaseDelaySecs;
		Pose2d robotPose = drivebase
			.getPose()
			.exp(
				new Twist2d(
					robotRelative.vxMetersPerSecond * dt,
					robotRelative.vyMetersPerSecond * dt,
					robotRelative.omegaRadiansPerSecond * dt
				)
			);
		Rotation2d heading = robotPose.getRotation();

		// Turret position on the field
		Translation2d turretOffsetField = new Translation2d(
			ShotingOnTheFlyConstants.robotToTurret.getX(),
			ShotingOnTheFlyConstants.robotToTurret.getY()
		).rotateBy(heading);
		Translation2d turretPos = robotPose.getTranslation().plus(turretOffsetField);

		// Turret field velocity = chassis velocity + omega x r
		ChassisSpeeds fieldVel = drivebase.getFieldVelocity();
		double omega = fieldVel.omegaRadiansPerSecond;
		double turretVx = fieldVel.vxMetersPerSecond - omega * turretOffsetField.getY();
		double turretVy = fieldVel.vyMetersPerSecond + omega * turretOffsetField.getX();

		Translation2d target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

		// Solve for the virtual launch point: ball lands where it would from turretPos + v * tof
		Translation2d virtualPos = turretPos;
		double distance = target.getDistance(turretPos);
		double tof = 0;
		for (int i = 0; i < ShotingOnTheFlyConstants.kLookaheadIterations; i++) {
			tof = timeOfFlightMap.get(distance);
			virtualPos = turretPos.plus(new Translation2d(turretVx * tof, turretVy * tof));
			double newDistance = target.getDistance(virtualPos);
			if (Math.abs(newDistance - distance) < 0.005) {
				distance = newDistance;
				break;
			}
			distance = newDistance;
		}

		// Turret: field aim angle -> robot relative -> raw turret angle
		Rotation2d fieldAim = target.minus(virtualPos).getAngle();
		double robotRelDeg = fieldAim.minus(heading).getDegrees();
		SmartDashboard.putNumber("SOTM/RobotRelAimDeg", robotRelDeg);
		publishTuningEntries();
		double trimDeg = SmartDashboard.getNumber("SOTM/TurretTrimDeg", ShotingOnTheFlyConstants.kTurretTrimDeg);
		double direction = SmartDashboard.getBoolean("SOTM/TurretFlipped", ShotingOnTheFlyConstants.kTurretFlipped)
			? -1.0
			: 1.0;
		double rawDeg = MathUtil.inputModulus(
			direction * (robotRelDeg - ShotingOnTheFlyConstants.kTurretZeroOffsetDeg) + trimDeg,
			-180.0,
			180.0
		);
		turretReachable =
			rawDeg >= ShotingOnTheFlyConstants.kTurretMinDeg && rawDeg <= ShotingOnTheFlyConstants.kTurretMaxDeg;
		turretSetpointDeg = clampToTurretRange(rawDeg);

		inRange = distance >= minDistance && distance <= maxDistance;
		if (SmartDashboard.getBoolean("SOTM/TuningMode", false)) {
			hoodSetpointDeg = SmartDashboard.getNumber("SOTM/TuningHoodDeg", 23);
			flywheelSetpointRPM = SmartDashboard.getNumber("SOTM/TuningRPM", 4000);
		} else {
			hoodSetpointDeg = hoodDegMap.get(distance);
			flywheelSetpointRPM = flywheelRPMMap.get(distance);
		}

		SmartDashboard.putNumber("SOTM/Distance", distance);
		SmartDashboard.putNumber("SOTM/TimeOfFlight", tof);
		SmartDashboard.putNumber("SOTM/TurretSetpointDeg", turretSetpointDeg);
		SmartDashboard.putNumber("SOTM/TurretActualDeg", turret.getRawAngle().in(Degrees));
		SmartDashboard.putNumber("SOTM/HoodSetpointDeg", hoodSetpointDeg);
		SmartDashboard.putNumber("SOTM/FlywheelSetpointRPM", flywheelSetpointRPM);
		SmartDashboard.putBoolean("SOTM/InRange", inRange);
		SmartDashboard.putBoolean("SOTM/TurretReachable", turretReachable);
		SmartDashboard.putBoolean("SOTM/Ready", isReady());
		drivebase.getSwerveDrive().field.getObject("SOTM/AimPoint").setPose(new Pose2d(virtualPos, fieldAim));
	}

	/** Clamp into the turret's range; if in the dead zone, park at whichever limit is angularly closer. */
	private static double clampToTurretRange(double rawDeg) {
		double min = ShotingOnTheFlyConstants.kTurretMinDeg;
		double max = ShotingOnTheFlyConstants.kTurretMaxDeg;
		if (rawDeg >= min && rawDeg <= max) return rawDeg;
		double distToMax = MathUtil.inputModulus(rawDeg - max, 0, 360);
		double distToMin = MathUtil.inputModulus(min - rawDeg, 0, 360);
		return distToMax < distToMin ? max : min;
	}

	/** True when a valid solution exists and every mechanism is at its setpoint. */
	public boolean isReady() {
		return (
			isScheduled() &&
			inRange &&
			turretReachable &&
			Math.abs(turret.getRawAngle().in(Degrees) - turretSetpointDeg) <
			ShotingOnTheFlyConstants.kTurretToleranceDeg &&
			Math.abs(hood.getAngle().in(Degrees) - hoodSetpointDeg) < ShotingOnTheFlyConstants.kHoodToleranceDeg &&
			Math.abs(shooter.getVelocity().in(RPM) - flywheelSetpointRPM) <
			ShotingOnTheFlyConstants.kShooterToleranceRPM
		);
	}

	public Trigger readyTrigger() {
		return new Trigger(this::isReady);
	}
}
