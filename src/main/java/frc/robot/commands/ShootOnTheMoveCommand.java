package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootingOnTheMoveConstants;
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
		(int) (0.1 / ShootingOnTheMoveConstants.loopPeriodSecs)
	);
	private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage(
		(int) (0.1 / ShootingOnTheMoveConstants.loopPeriodSecs)
	);

	private Rotation2d lastTurretAngle;
	private double lastHoodAngle;
	private Rotation2d turretAngle;
	// Why is it NaN?
	private double hoodAngle = Double.NaN;
	private double turretVelocity;
	private double hoodVelocity;

	public ShootOnTheMoveCommand(SwerveSubsystem drivetrain, ScoringSystem superstructure) {
		this.drivetrain = drivetrain;
		this.superstructure = superstructure;
	}

	@Override
	public void initialize() {
		super.initialize();

		lastHoodAngle = superstructure.getHoodAngle().in(Degrees);
		lastTurretAngle = Rotation2d.fromRotations(superstructure.getTurretAngle().in(Degrees));

		superstructure.aimDynamicCommand(
			() -> RPM.of(ShootingOnTheMoveConstants.flywheelRPM),
			() -> Rotations.of(lastTurretAngle.getRotations()),
			() -> Rotations.of(lastHoodAngle)
		).schedule();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void execute() {
		Pose2d estimatedPose = estimatePoseWithPhaseDelay();

		// Designate desired target
		Translation2d target = getCurrentTarget();

		// Calculate distance from turret to target
		Pose2d turretPosition = estimatedPose.transformBy(ShootingOnTheMoveConstants.robotToTurret.toTransform2d());
		double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());
		SmartDashboard.putNumber("Turret to Target Distance", turretToTargetDistance);

		// Calculate field relative chassis velocity
		ChassisSpeeds chassisVel = drivetrain.getFieldVelocity();

		// Kinematic approach
		aimHandler(turretPosition, target, chassisVel);

		AngularVelocity desiredRPM = isReadyToShoot() ? RPM.of(ShootingOnTheMoveConstants.flywheelRPM) : RPM.of(0);

		superstructure.setShooterSetpoints(
			desiredRPM,
			Rotations.of(turretAngle.getRotations()),
			Rotations.of(hoodAngle)
		);
	}

	private boolean inLeftNeutralZone() {
		return ScoringSystem.CustomTriggers.leftNeutralZone.getTrigger().getAsBoolean();
	}

	private boolean inRightNeutralZone() {
		return ScoringSystem.CustomTriggers.rightNeutralZone.getTrigger().getAsBoolean();
	}

	private boolean inScoringZone() {
		return ScoringSystem.CustomTriggers.scoringZone.getTrigger().getAsBoolean() ||
			ScoringSystem.CustomTriggers.bumpZone.getTrigger().getAsBoolean();
	}

	private boolean isReadyToShoot() {
		return inScoringZone();  // require "&& the angle is satisfactory"? or does it just update quick enough...
	}

	public void aimHandler(Pose2d turretPose, Translation2d target, ChassisSpeeds chassisVel) {
		if (inLeftNeutralZone() || inRightNeutralZone()) {
			hoardFuelInZoneAim(turretPose, chassisVel);
			return;
		}

		// Kinematic approach
		kinematicLaunchingParametersAim(turretPose, target, chassisVel);
	}

	public void hoardFuelInZoneAim(Pose2d turretPose, ChassisSpeeds chassisVel) {
		double y = 3.0;
		if (turretPose.getTranslation().getY() < 0) {
			y *= -1.0;
		}
		Translation2d target = new Translation2d(-5.0, y);

		kinematicLaunchingParametersAim(turretPose, target, chassisVel);
	}

	private Pose2d estimatePoseWithPhaseDelay() {
		Pose2d pose = drivetrain.getPose();
		ChassisSpeeds vel = drivetrain.getRobotVelocity();
		return pose.exp(new Twist2d(
			vel.vxMetersPerSecond * ShootingOnTheMoveConstants.phaseDelay,
			vel.vyMetersPerSecond * ShootingOnTheMoveConstants.phaseDelay,
			vel.omegaRadiansPerSecond * ShootingOnTheMoveConstants.phaseDelay
		));
	}

	private Translation2d getCurrentTarget() {
		if (inScoringZone()) {
			return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
		}
		else if (inLeftNeutralZone()) {
			// magic numbers?
			return AllianceFlipUtil.apply(FieldConstants.LeftBump.nearRightCorner.plus(new Translation2d(0, Inches.of(36.5).in(Meters))));
		}
		else if (inRightNeutralZone()) {
			return AllianceFlipUtil.apply(FieldConstants.RightBump.nearRightCorner.plus(new Translation2d(0, Inches.of(36.5).in(Meters))));
		}
		else {
			return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
		}
	}


	// Pure solver: given horizontal distance and launch speed, returns hood angle in radians.
	// Returns NaN if target is unreachable (imaginary discriminant).
	private double solveHoodAngle(double adjustedDist, double v0) {
		double g = ShootingOnTheMoveConstants.gAcceleration;
		double v2 = v0 * v0;
		double verticalDist = FieldConstants.Hub.height;
		double discr = v2*v2 - g * (g*adjustedDist*adjustedDist + 2*verticalDist*v2);

		if (discr < 0) return Double.NaN;  // Imaginary solutions; cannot reach target

		// High arc (additive) solution
		return Math.atan((v2 + Math.sqrt(discr)) / (g * adjustedDist));
	}

	// MY IDEA: Kinematics?
	// After watching some FRC footage, I noticed our shooting was kind of weak
	// I watched some clips from an FRC Istanbul regional final this year, and noticed their arc was forgivably parabolic.
	// Hopefully this helps.
	private void kinematicLaunchingParametersAim(Pose2d turretPose, Translation2d target, ChassisSpeeds chassisVel) {
		double v0 = ShootingOnTheMoveConstants.flywheelRPM * (2 * Math.PI / 60.0) * ShootingOnTheMoveConstants.flywheelRadiusMeters;  // m/s, maybe this needs an efficiency factor
		
		double dx = target.getX() - turretPose.getTranslation().getX();
		double dy = target.getY() - turretPose.getTranslation().getY();
		double horizontalDist = Math.sqrt(dx*dx + dy*dy);

		// first-pass flight time approximation
		double estimatedFlightTime = horizontalDist / v0;  // initial rough estimate
		double hoodSolution = Double.NaN;
		double adx = dx, ady = dy;

		// iteratively estimate flight time.
		// by default, aimIterations is set to 2.
		for (int i = 0; i < ShootingOnTheMoveConstants.aimIterations; i++) {
			adx = dx - chassisVel.vxMetersPerSecond * estimatedFlightTime;
			ady = dy - chassisVel.vyMetersPerSecond * estimatedFlightTime;
			double adjustedDist = Math.sqrt(adx*adx + ady*ady);

			hoodSolution = solveHoodAngle(adjustedDist, v0);
			if (Double.isNaN(hoodSolution)) return;  // Unreachable, so hold last known good angles

			// Refine flight time using actual horizontal velocity component
			estimatedFlightTime = adjustedDist / (v0 * Math.cos(hoodSolution));
		}

		hoodAngle = hoodSolution;

		Pose2d robotPose = drivetrain.getPose();
		turretAngle = new Translation2d(adx, ady).getAngle().minus(robotPose.getRotation());

		// why is this necessary for every loop?  why not on init?
		if (lastTurretAngle == null) lastTurretAngle = turretAngle;
		if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

		// Should these be abstracted into two calls of one "calcFilteredDerivative" function?
		turretVelocity = turretAngleFilter.calculate((turretAngle.minus(lastTurretAngle)).getRadians() / ShootingOnTheMoveConstants.loopPeriodSecs);
		hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / ShootingOnTheMoveConstants.loopPeriodSecs);

		lastTurretAngle = turretAngle;
		lastHoodAngle = hoodAngle;
	}

}
