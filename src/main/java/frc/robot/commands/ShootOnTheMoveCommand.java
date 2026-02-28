package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ShotingOnTheFlyConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.systems.ScoringSystem;
import frc.robot.utils.field.AllianceFlipUtil;
import frc.robot.utils.field.FieldConstants;
import frc.robot.utils.field.GeomUtil;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({ GeomUtil.class })
public class ShootOnTheMoveCommand extends Command {

	private final SwerveSubsystem drivetrain;
	private final ScoringSystem superstructure;

	private final LinearFilter turretAngleFilter = LinearFilter.movingAverage(
		(int) (0.1 / ShotingOnTheFlyConstants.loopPeriodSecs)
	);
	private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage(
		(int) (0.1 / ShotingOnTheFlyConstants.loopPeriodSecs)
	);

	private Rotation2d lastTurretAngle;
	private double lastHoodAngle;
	private Rotation2d turretAngle;
	private double hoodAngle = Double.NaN;
	private double turretVelocity;
	private double hoodVelocity;
	private AngularVelocity lastShootSpeed;

	public record LaunchingParameters(
		boolean isValid,
		Rotation2d turretAngle,
		double turretVelocity,
		double hoodAngle,
		double hoodVelocity,
		double flywheelSpeed
	) {}

	// Cache parameters
	private LaunchingParameters latestParameters = null;

	private static double minDistance;
	private static double maxDistance;
	private static double phaseDelay;
	private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap = new InterpolatingTreeMap<>(
		InverseInterpolator.forDouble(),
		Rotation2d::interpolate
	);
	private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
	private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

	static {
		minDistance = 1.34;
		maxDistance = 5.60;
		phaseDelay = 0.03;

		launchHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
		launchHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
		launchHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
		launchHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
		launchHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
		launchHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
		launchHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
		launchHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
		launchHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
		launchHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

		launchFlywheelSpeedMap.put(Inches.of(118.51).in(Meters), 7500.0);
		launchFlywheelSpeedMap.put(Inches.of(111.51).in(Meters), 7000.0);
		launchFlywheelSpeedMap.put(Inches.of(97.51).in(Meters), 6800.0);
		launchFlywheelSpeedMap.put(Inches.of(87.51).in(Meters), 6600.0);
		launchFlywheelSpeedMap.put(Inches.of(77.51).in(Meters), 6100.0);

		timeOfFlightMap.put(5.68, 1.16);
		timeOfFlightMap.put(4.55, 1.12);
		timeOfFlightMap.put(3.15, 1.11);
		timeOfFlightMap.put(1.88, 1.09);
		timeOfFlightMap.put(1.38, 0.90);
	}

	public ShootOnTheMoveCommand(SwerveSubsystem drivetrain, ScoringSystem superstructure) {
		this.drivetrain = drivetrain;
		this.superstructure = superstructure;
	}

	@Override
	public void initialize() {
		super.initialize();

		lastHoodAngle = superstructure.getHoodAngle().in(Units.Degrees);
		lastTurretAngle = Rotation2d.fromRotations(superstructure.getTurretAngle().in(Units.Degrees));
		lastShootSpeed = superstructure.getShooterSpeed();

		// TODO: when this current command ends, we should probably cancel the dynamic
		// aim command
		superstructure
			.aimDynamicCommand(
				() -> {
					return this.lastShootSpeed;
				},
				() -> {
					if (this.lastTurretAngle.getRotations() < 0) {
						return Rotations.of(this.lastTurretAngle.getRotations() + 1);
					} else {
						return Rotations.of(this.lastTurretAngle.getRotations());
					}
				},
				() -> {
					return Rotations.of(this.lastHoodAngle);
				}
			)
			.schedule();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void execute() {
		// Calculate estimated pose while accounting for phase delay
		Pose2d estimatedPose = drivetrain.getPose();
		ChassisSpeeds robotRelativeVelocity = drivetrain.getRobotVelocity();
		estimatedPose = estimatedPose.exp(
			new Twist2d(
				robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
				robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
				robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay
			)
		);

		// Calculate distance from turret to target
		Pose2d turretPosition = estimatedPose.transformBy(ShotingOnTheFlyConstants.robotToTurret.toTransform2d());

		// Designate desired target

		Translation2d target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

		if (ScoringSystem.CustomTriggers.scoringZone.getTrigger().getAsBoolean()) {
			target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
		}

		if (ScoringSystem.CustomTriggers.bumpZone.getTrigger().getAsBoolean()) {
			target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
		}

		if (ScoringSystem.CustomTriggers.leftNeutralZone.getTrigger().getAsBoolean()) {
			target = AllianceFlipUtil.apply(
				FieldConstants.LeftBump.nearRightCorner.plus(new Translation2d(0, Inches.of(36.5).in(Meters)))
			);
		}

		if (ScoringSystem.CustomTriggers.rightNeutralZone.getTrigger().getAsBoolean()) {
			target = AllianceFlipUtil.apply(
				FieldConstants.RightBump.nearRightCorner.plus(new Translation2d(0, Inches.of(36.5).in(Meters)))
			);
		}

		// if (inScoringZone(turretPosition).getAsBoolean()) {
		//   target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
		// }

		// if (inRightNeutralZone(turretPosition).getAsBoolean()) {
		//   target =
		//       AllianceFlipUtil.apply(
		//           FieldConstants.RightBump.nearRightCorner.plus(
		//               new Translation2d(0, Inches.of(36.5).in(Meters))));
		// }

		// if (inLeftNeutralZone(turretPosition).getAsBoolean()) {
		//   target =
		//       AllianceFlipUtil.apply(
		//           FieldConstants.LeftBump.nearRightCorner.plus(
		//               new Translation2d(0, Inches.of(36.5).in(Meters))));
		// }

		double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

		// Calculate field relative turret velocity
		ChassisSpeeds robotVelocity = drivetrain.getFieldVelocity();
		double robotAngle = estimatedPose.getRotation().getRadians();
		double turretVelocityX =
			robotVelocity.vxMetersPerSecond +
			robotVelocity.omegaRadiansPerSecond *
			(ShotingOnTheFlyConstants.robotToTurret.getY() * Math.cos(robotAngle) -
				ShotingOnTheFlyConstants.robotToTurret.getX() * Math.sin(robotAngle));
		double turretVelocityY =
			robotVelocity.vyMetersPerSecond +
			robotVelocity.omegaRadiansPerSecond *
			(ShotingOnTheFlyConstants.robotToTurret.getX() * Math.cos(robotAngle) -
				ShotingOnTheFlyConstants.robotToTurret.getY() * Math.sin(robotAngle));

		// Account for imparted velocity by robot (turret) to offset
		double timeOfFlight;
		Pose2d lookaheadPose = turretPosition;
		double lookaheadTurretToTargetDistance = turretToTargetDistance;
		for (int i = 0; i < 20; i++) {
			timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
			double offsetX = turretVelocityX * timeOfFlight;
			double offsetY = turretVelocityY * timeOfFlight;
			lookaheadPose = new Pose2d(
				turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
				turretPosition.getRotation()
			);
			lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
		}

		// Calculate parameters accounted for imparted velocity
		turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle().minus(estimatedPose.getRotation());
		hoodAngle = launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
		if (lastTurretAngle == null) lastTurretAngle = turretAngle;
		if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
		turretVelocity = turretAngleFilter.calculate(
			turretAngle.minus(lastTurretAngle).getRadians() / ShotingOnTheFlyConstants.loopPeriodSecs
		);
		hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / ShotingOnTheFlyConstants.loopPeriodSecs);
		lastTurretAngle = turretAngle;
		lastHoodAngle = hoodAngle;
		latestParameters = new LaunchingParameters(
			lookaheadTurretToTargetDistance >= minDistance && lookaheadTurretToTargetDistance <= maxDistance,
			turretAngle,
			turretVelocity,
			hoodAngle,
			hoodVelocity,
			launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance)
		);

		lastShootSpeed = RPM.of(launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

		superstructure.setShooterSetpoints(
			RPM.of(launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance)),
			Rotations.of(turretAngle.getRotations()),
			Rotations.of(hoodAngle)
		);
	}

	public void clearLaunchingParameters() {
		latestParameters = null;
	}

	public Trigger inScoringZone(Pose2d turretPose) {
		return new Trigger(() ->
			new Rectangle2d(
				AllianceFlipUtil.apply(new Translation2d(0, 0)),
				AllianceFlipUtil.apply(
					new Translation2d(FieldConstants.LinesVertical.starting, FieldConstants.fieldWidth)
				)
			).contains(turretPose.getTranslation())
		);
	}

	public Trigger inRightNeutralZone(Pose2d turretPose) {
		return new Trigger(() ->
			new Rectangle2d(
				AllianceFlipUtil.apply(new Translation2d(FieldConstants.LinesVertical.starting, 0)),
				AllianceFlipUtil.apply(
					new Translation2d(
						FieldConstants.LinesVertical.oppAllianceZone,
						FieldConstants.LinesHorizontal.center
					)
				)
			).contains(turretPose.getTranslation())
		);
	}

	public Trigger inLeftNeutralZone(Pose2d turretPose) {
		return new Trigger(() ->
			new Rectangle2d(
				AllianceFlipUtil.apply(
					new Translation2d(FieldConstants.LinesVertical.starting, FieldConstants.LinesHorizontal.center)
				),
				AllianceFlipUtil.apply(
					new Translation2d(FieldConstants.LinesVertical.oppAllianceZone, FieldConstants.fieldWidth)
				)
			).contains(turretPose.getTranslation())
		);
	}
}
