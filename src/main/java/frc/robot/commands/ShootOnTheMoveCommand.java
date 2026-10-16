package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
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

	// Passing (neutral zone -> our alliance zone) uses its own table
	private static final InterpolatingDoubleTreeMap passRPMMap = new InterpolatingDoubleTreeMap();
	private static final InterpolatingDoubleTreeMap passHoodDegMap = new InterpolatingDoubleTreeMap();
	private static final InterpolatingDoubleTreeMap passTimeOfFlightMap = new InterpolatingDoubleTreeMap();
	private static final double passMinDistance;
	private static final double passMaxDistance;

	static {
		double[][] table = ShotingOnTheFlyConstants.kShotTable;
		for (double[] row : table) {
			flywheelRPMMap.put(row[0], row[1]);
			hoodDegMap.put(row[0], row[2]);
			timeOfFlightMap.put(row[0], row[3]);
		}
		minDistance = table[0][0];
		maxDistance = table[table.length - 1][0];

		double[][] passTable = ShotingOnTheFlyConstants.kPassTable;
		for (double[] row : passTable) {
			passRPMMap.put(row[0], row[1]);
			passHoodDegMap.put(row[0], row[2]);
			passTimeOfFlightMap.put(row[0], row[3]);
		}
		passMinDistance = passTable[0][0];
		passMaxDistance = passTable[passTable.length - 1][0];

		publishTuningEntries();
	}

	/** Creates the tuning entries if they are missing, without overwriting values typed on the dashboard. */
	private static void publishTuningEntries() {
		SmartDashboard.setDefaultBoolean("SOTM/TuningMode", false);
		SmartDashboard.setDefaultNumber("SOTM/TuningRPM", 4000);
		SmartDashboard.setDefaultNumber("SOTM/TuningHoodDeg", 23);
		SmartDashboard.setDefaultNumber("SOTM/TurretTrimDeg", ShotingOnTheFlyConstants.kTurretTrimDeg);
		SmartDashboard.setDefaultBoolean("SOTM/TurretFlipped", ShotingOnTheFlyConstants.kTurretFlipped);
		SmartDashboard.setDefaultNumber("SOTM/TurretLeadSecs", ShotingOnTheFlyConstants.kTurretLeadSecs);
		SmartDashboard.setDefaultNumber("SOTM/DragConstant", ShotingOnTheFlyConstants.kLinearDragTimeConstant);
		SmartDashboard.setDefaultNumber("SOTM/FlywheelOffsetRPM", 0.0);
		SmartDashboard.setDefaultBoolean("SOTM/PassEnabled", ShotingOnTheFlyConstants.kPassEnabled);
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
	private boolean passing = false; // true = in the neutral zone, lobbing to our alliance zone instead of the hub

	// Turret lead: estimate how fast the aim angle is changing and point a little ahead of it
	private final LinearFilter turretRateFilter = LinearFilter.movingAverage(5);
	private double lastAimDeg = Double.NaN;
	private double aimDeg; // where the turret should point right now, before lead
	private double turretToleranceDeg = ShotingOnTheFlyConstants.kTurretToleranceMinDeg; // set by distance each loop

	// isScheduled() is false when this runs inside an auto group, so track "running" by when we last calculated
	private double lastCalculateTime = Double.NEGATIVE_INFINITY;

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
		aimDeg = turretSetpointDeg;
		hoodSetpointDeg = hood.getAngle().in(Degrees);
		flywheelSetpointRPM = 0;

		// Parallel groups execute children in order, so the solver runs before the mechanisms read it.
		addCommands(
			Commands.runOnce(this::resetLead),
			Commands.run(this::calculate),
			turret.setAngleDynamic(() -> Degrees.of(turretSetpointDeg)),
			hood.setAngleDynamic(() -> Degrees.of(hoodSetpointDeg)),
			shooter.setVelocityDynamic(() -> RPM.of(flywheelSetpointRPM))
		);
		setName("ShootOnTheMove");
	}

	private void calculate() {
		lastCalculateTime = Timer.getFPGATimestamp();
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

		passing = shouldPass(robotPose.getTranslation());
		Translation2d target = passing
			? passTarget(robotPose.getTranslation())
			: AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
		InterpolatingDoubleTreeMap tofMap = passing ? passTimeOfFlightMap : timeOfFlightMap;

		// Solve for the virtual launch point: ball lands where it would from turretPos + v * tof
		Translation2d virtualPos = turretPos;
		double distance = target.getDistance(turretPos);
		double tof = 0;
		double dragK = SmartDashboard.getNumber("SOTM/DragConstant", ShotingOnTheFlyConstants.kLinearDragTimeConstant);
		double effectiveTof = 0;
		for (int i = 0; i < ShotingOnTheFlyConstants.kLookaheadIterations; i++) {
			tof = tofMap.get(distance);
			// Drag: the ball doesn't keep all of the robot's speed for the whole flight (6328's LaunchCalculator)
			effectiveTof = dragK > 1e-6 ? (1 - Math.exp(-tof * dragK)) / dragK : tof;
			virtualPos = turretPos.plus(new Translation2d(turretVx * effectiveTof, turretVy * effectiveTof));
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
		// Turret lead: aim ahead by (how fast the aim angle is changing) x kTurretLeadSecs
		double rateDegPerSec = Double.isNaN(lastAimDeg)
			? 0
			: MathUtil.inputModulus(rawDeg - lastAimDeg, -180, 180) / ShotingOnTheFlyConstants.loopPeriodSecs;
		lastAimDeg = rawDeg;
		double smoothedRate = turretRateFilter.calculate(rateDegPerSec);
		aimDeg = clampToTurretRange(rawDeg);
		double leadSecs = SmartDashboard.getNumber("SOTM/TurretLeadSecs", ShotingOnTheFlyConstants.kTurretLeadSecs);
		turretSetpointDeg = clampToTurretRange(rawDeg + smoothedRate * leadSecs);

		inRange = passing
			? distance >= passMinDistance && distance <= passMaxDistance
			: distance >= minDistance && distance <= maxDistance;
		if (SmartDashboard.getBoolean("SOTM/TuningMode", false)) {
			hoodSetpointDeg = SmartDashboard.getNumber("SOTM/TuningHoodDeg", 23);
			flywheelSetpointRPM = SmartDashboard.getNumber("SOTM/TuningRPM", 4000);
		} else if (passing) {
			hoodSetpointDeg = passHoodDegMap.get(distance);
			flywheelSetpointRPM = passRPMMap.get(distance);
		} else {
			hoodSetpointDeg = hoodDegMap.get(distance);
			flywheelSetpointRPM =
				flywheelRPMMap.get(distance) + SmartDashboard.getNumber("SOTM/FlywheelOffsetRPM", 0.0);
		}

		turretToleranceDeg = turretToleranceDeg(distance, passing);

		SmartDashboard.putNumber("SOTM/Distance", distance);
		SmartDashboard.putNumber("SOTM/TurretToleranceDeg", turretToleranceDeg);
		SmartDashboard.putNumber("SOTM/TimeOfFlight", tof);
		SmartDashboard.putNumber("SOTM/EffectiveTOF", effectiveTof);
		SmartDashboard.putNumber("SOTM/TurretSetpointDeg", turretSetpointDeg);
		SmartDashboard.putNumber("SOTM/TurretAimDeg", aimDeg);
		SmartDashboard.putNumber("SOTM/TurretActualDeg", turret.getRawAngle().in(Degrees));
		SmartDashboard.putNumber("SOTM/HoodSetpointDeg", hoodSetpointDeg);
		SmartDashboard.putNumber("SOTM/FlywheelSetpointRPM", flywheelSetpointRPM);
		SmartDashboard.putBoolean("SOTM/InRange", inRange);
		SmartDashboard.putBoolean("SOTM/TurretReachable", turretReachable);
		SmartDashboard.putBoolean("SOTM/TurretOnTarget", turretOnTarget());
		SmartDashboard.putBoolean("SOTM/HoodOnTarget", hoodOnTarget());
		SmartDashboard.putBoolean("SOTM/FlywheelAtSpeed", flywheelAtSpeed());
		SmartDashboard.putBoolean("SOTM/Ready", isReady());
		SmartDashboard.putBoolean("SOTM/Passing", passing);
		drivebase.getSwerveDrive().field.getObject("SOTM/AimPoint").setPose(new Pose2d(virtualPos, fieldAim));
		drivebase.getSwerveDrive().field.getObject("SOTM/Target").setPose(new Pose2d(target, Rotation2d.kZero));
	}

	/** Past our alliance zone line (and passing is on) -> pass. Hysteresis keeps it from flickering on the line. */
	private boolean shouldPass(Translation2d robotPos) {
		if (!SmartDashboard.getBoolean("SOTM/PassEnabled", ShotingOnTheFlyConstants.kPassEnabled)) return false;
		double x = AllianceFlipUtil.applyX(robotPos.getX()); // distance from our alliance wall
		double line = FieldConstants.LinesVertical.allianceZone + ShotingOnTheFlyConstants.kPassStartOffsetM;
		double band = ShotingOnTheFlyConstants.kPassHysteresisM;
		return passing ? x > line - band : x > line + band;
	}

	/** Pass spot in our alliance zone, on the robot's side of the field. */
	private static Translation2d passTarget(Translation2d robotPos) {
		boolean rightSide = AllianceFlipUtil.applyY(robotPos.getY()) < FieldConstants.LinesHorizontal.center;
		return AllianceFlipUtil.apply(
			rightSide ? ShotingOnTheFlyConstants.kPassTargetRight : ShotingOnTheFlyConstants.kPassTargetLeft
		);
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

	/** Forget the old aim so the lead doesn't jump when tracking starts again. */
	private void resetLead() {
		lastAimDeg = Double.NaN;
		turretRateFilter.reset();
	}

	/** Compared against the real aim angle, not the led setpoint, so the lead doesn't block Ready. */
	private boolean turretOnTarget() {
		return Math.abs(turret.getRawAngle().in(Degrees) - aimDeg) < turretToleranceDeg;
	}

	/** Allowed turret error (deg) so the ball still lands within the aim radius of the target at this distance. */
	private static double turretToleranceDeg(double distance, boolean passing) {
		double radius = passing ? ShotingOnTheFlyConstants.kPassAimRadiusM : ShotingOnTheFlyConstants.kHubAimRadiusM;
		return MathUtil.clamp(
			Math.toDegrees(Math.atan2(radius, distance)),
			ShotingOnTheFlyConstants.kTurretToleranceMinDeg,
			ShotingOnTheFlyConstants.kTurretToleranceMaxDeg
		);
	}

	private boolean hoodOnTarget() {
		return Math.abs(hood.getAngle().in(Degrees) - hoodSetpointDeg) < ShotingOnTheFlyConstants.kHoodToleranceDeg;
	}

	private boolean flywheelAtSpeed() {
		return (
			Math.abs(shooter.getVelocity().in(RPM) - flywheelSetpointRPM) <
			ShotingOnTheFlyConstants.kShooterToleranceRPM
		);
	}

	/** True if the solver ran in the last 0.1 s, whether scheduled alone or inside a group. */
	private boolean isRunning() {
		return Timer.getFPGATimestamp() - lastCalculateTime < 0.1;
	}

	/** True when a valid solution exists and every mechanism is close enough to fire. */
	public boolean isReady() {
		return isRunning() && inRange && turretReachable && turretOnTarget() && hoodOnTarget() && flywheelAtSpeed();
	}

	/** Operator nudge: add RPM to every table shot (negative = less). Shared by all SOTM commands; resets on reboot. */
	public static void nudgeFlywheel(double rpm) {
		publishTuningEntries();
		SmartDashboard.putNumber(
			"SOTM/FlywheelOffsetRPM",
			SmartDashboard.getNumber("SOTM/FlywheelOffsetRPM", 0.0) + rpm
		);
	}

	public Trigger readyTrigger() {
		return new Trigger(this::isReady);
	}
}
