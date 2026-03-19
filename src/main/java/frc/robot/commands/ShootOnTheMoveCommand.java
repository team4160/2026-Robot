package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

	private Rotation2d turretAngle;
	private double hoodAngle;

	public ShootOnTheMoveCommand(SwerveSubsystem drivetrain, ScoringSystem superstructure) {
		this.drivetrain = drivetrain;
		this.superstructure = superstructure;
		addRequirements(superstructure.shooter, superstructure.turret, superstructure.hood);
	}

	// create it so it can be init'ed and ended.
	private Command aimDynamicInstance;

	@Override
	public void initialize() {
		super.initialize();

		hoodAngle = superstructure.getHoodAngle().in(Degrees);
		turretAngle = Rotation2d.fromRotations(superstructure.getTurretAngle().in(Degrees));

		superstructure.setShooterSetpoints(
			RPM.of(0.0),
			Rotations.of(turretAngle.getRotations()),
			Rotations.of(hoodAngle)
		);

		// Command.schedule is deprecated
		// now have to use CommandScheduler.getInstance.schedule(cmd(...).asProxy());

		aimDynamicInstance = superstructure
			.aimDynamicCommand(
				superstructure::getTargetShooterSpeed,
				superstructure::getTargetTurretAngle,
				superstructure::getTargetHoodAngle
			)
			.asProxy();

		CommandScheduler.getInstance().schedule(aimDynamicInstance);
	}

	@Override
	public void end(boolean interrupted) {
		CommandScheduler.getInstance().cancel(aimDynamicInstance);
		// return control to the ops.
		// default commands take back control immediately, now.
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
		ChassisSpeeds chassisVel = drivetrain.getFieldVelocity();
		Pose2d turretPosition = estimatedPose.transformBy(ShootingOnTheMoveConstants.robotToTurret.toTransform2d());

		// double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());
		// SmartDashboard.putNumber("Turret to Target Distance", turretToTargetDistance);

		// Kinematic approach
		aimHandler(turretPosition, target, chassisVel);

		AngularVelocity desiredRPM = inScoringZone()
			? RPM.of(ShootingOnTheMoveConstants.flywheelRPM)
			: RPM.of(ShootingOnTheMoveConstants.flywheelNeutralZoneRPM);

		// shoot fuel slower when hoarding
		// to avoid accidentally bouncing some out of bounds.

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
		return (
			ScoringSystem.CustomTriggers.scoringZone.getTrigger().getAsBoolean() ||
			ScoringSystem.CustomTriggers.bumpZone.getTrigger().getAsBoolean()
		);
	}

	public void aimHandler(Pose2d turretPose, Translation2d target, ChassisSpeeds chassisVel) {
		if (inLeftNeutralZone() || inRightNeutralZone()) {
			kinematicLaunchingParametersAim(
				turretPose,
				target,
				0.1,
				chassisVel,
				ShootingOnTheMoveConstants.flywheelNeutralZoneRPM
			);
			return;
		}

		// Kinematic approach
		kinematicLaunchingParametersAim(
			turretPose,
			target,
			FieldConstants.Hub.height,
			chassisVel,
			ShootingOnTheMoveConstants.flywheelRPM
		);
	}

	private Pose2d estimatePoseWithPhaseDelay() {
		Pose2d pose = drivetrain.getPose();
		ChassisSpeeds vel = drivetrain.getRobotVelocity();
		return pose.exp(
			new Twist2d(
				vel.vxMetersPerSecond * ShootingOnTheMoveConstants.phaseDelay,
				vel.vyMetersPerSecond * ShootingOnTheMoveConstants.phaseDelay,
				vel.omegaRadiansPerSecond * ShootingOnTheMoveConstants.phaseDelay
			)
		);
	}

	private Translation2d getCurrentTarget() {
		// in NZ, aim at tower Y pos, but depot X pos
		// in SZ, aim for hub opening
		if (inScoringZone()) {
			return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
		} else if (inLeftNeutralZone()) {
			// aiming towards our left empty zone
			return AllianceFlipUtil.apply(
				FieldConstants.Tower.centerPoint.plus(new Translation2d(Units.metersToInches(-2.5), 0.0))
			);
		} else if (inRightNeutralZone()) {
			// or our right, depending on NZ placement
			return AllianceFlipUtil.apply(
				FieldConstants.Tower.centerPoint.plus(new Translation2d(Units.metersToInches(2.5), 0.0))
			);
		} else {
			return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
		}
	}

	// Pure solver: given horizontal distance and launch speed, returns hood angle in radians.
	// Returns NaN if target is unreachable (imaginary discriminant).
	private double solveHoodAngle(double adjustedDist, double v0, double verticalDist) {
		double g = ShootingOnTheMoveConstants.gAcceleration;
		double v2 = v0 * v0;
		double discr = v2 * v2 - g * (g * adjustedDist * adjustedDist + 2 * verticalDist * v2);

		if (discr < 0) return Double.NaN; // Imaginary solutions; cannot reach target

		// High arc (additive) solution
		return Math.atan((v2 + Math.sqrt(discr)) / (g * adjustedDist));
	}

	// MY IDEA: Kinematics?
	// After watching some FRC footage, I noticed our shooting was kind of weak
	// I watched some clips from a 2026 FRC Istanbul regional final, and noticed their arc was forgivably parabolic.
	private void kinematicLaunchingParametersAim(
		Pose2d turretPose,
		Translation2d target,
		double h,
		ChassisSpeeds chassisVel,
		double desiredRPM
	) {
		double v0 =
			desiredRPM *
			(Math.PI / 30.0) *
			ShootingOnTheMoveConstants.flywheelRadiusMeters *
			ShootingOnTheMoveConstants.RPMOverapproxFactor; // m/s, maybe this needs an efficiency factor

		double dx = target.getX() - turretPose.getTranslation().getX();
		double dy = target.getY() - turretPose.getTranslation().getY();
		double horizontalDist = Math.sqrt(dx * dx + dy * dy);

		// first-pass flight time approximation
		double estimatedFlightTime = horizontalDist / v0; // initial rough estimate
		double hoodSolution = Double.NaN;
		double adx = dx,
			ady = dy;

		// iteratively estimate flight time.
		// by default, aimIterations is set to 2.
		for (int i = 0; i < ShootingOnTheMoveConstants.aimIterations; i++) {
			adx = dx - chassisVel.vxMetersPerSecond * estimatedFlightTime;
			ady = dy - chassisVel.vyMetersPerSecond * estimatedFlightTime;
			double adjustedDist = Math.sqrt(adx * adx + ady * ady);

			hoodSolution = solveHoodAngle(adjustedDist, v0, h);
			if (Double.isNaN(hoodSolution)) return; // Unreachable, so hold last known good angles

			// Refine flight time using actual horizontal velocity component
			estimatedFlightTime = adjustedDist / (v0 * Math.cos(hoodSolution));
		}

		hoodAngle = hoodSolution;

		Pose2d robotPose = drivetrain.getPose();
		turretAngle = new Translation2d(-adx, -ady).getAngle().minus(robotPose.getRotation());
	}
}
