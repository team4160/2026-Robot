package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.ShootingOnTheMoveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.field.AllianceFlipUtil;
import frc.robot.utils.field.FieldConstants;
import frc.robot.utils.field.GeomUtil;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({ GeomUtil.class })
public class ShootOnTheMoveCommand extends Command {

	private final SwerveSubsystem drivetrain;
	private final RobotContainer rc;

	// Tracks current desired angles, RPM so the dynamic command supplier stays live
	private Rotation2d turretAngle;
	private double hoodAngle;
	private AngularVelocity currentRPM;

	// Scheduled once in initialize(), cancelled in end()
	private Command aimDynamicInstance;

	public ShootOnTheMoveCommand(SwerveSubsystem drivetrain, RobotContainer rc) {
		this.drivetrain = drivetrain;
		this.rc = rc;
		addRequirements(rc.getShooter(), rc.getTurret(), rc.getHood());
	}

	@Override
	public void initialize() {
		super.initialize();

		// Seed angles from current mechanism positions
		hoodAngle = rc.getHood().getAngle().in(Degrees);
		turretAngle = Rotation2d.fromDegrees(rc.getTurret().getRawAngle().in(Degrees));
		currentRPM = RPM.of(0);

		// Schedule the dynamic command once; suppliers read live fields each loop.
		// Note: Command.schedule() is deprecated since 2025; use CommandScheduler directly.
		aimDynamicInstance = Commands.parallel(
			rc
				.getShooter()
				.setVelocityDynamic(() -> currentRPM)
				.asProxy(),
			rc
				.getTurret()
				.setAngleDynamic(() -> turretAngle.getMeasure())
				.asProxy(),
			rc
				.getHood()
				.setAngleDynamic(() -> Degrees.of(hoodAngle))
				.asProxy()
		);
		CommandScheduler.getInstance().schedule(aimDynamicInstance);
	}

	@Override
	public void end(boolean interrupted) {
		// Cancel the proxied dynamic command so default commands take back over
		CommandScheduler.getInstance().cancel(aimDynamicInstance);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void execute() {
		Pose2d estimatedPose = estimatePoseWithPhaseDelay();

		// Designate desired target based on current zone
		Translation2d target = getCurrentTarget();
		ChassisSpeeds chassisVel = drivetrain.getFieldVelocity();
		Pose2d turretPosition = estimatedPose.transformBy(ShootingOnTheMoveConstants.robotToTurret.toTransform2d());

		// Kinematic approach updates turretAngle and hoodAngle fields
		aimHandler(turretPosition, target, chassisVel);

		// Shoot fuel slower when hoarding to avoid bouncing some out of bounds
		currentRPM = inScoringZone()
			? RPM.of(ShootingOnTheMoveConstants.flywheelRPM)
			: RPM.of(ShootingOnTheMoveConstants.flywheelNeutralZoneRPM);

		// aimDynamicInstance reads turretAngle, hoodAngle, and currentRPM
		// through its suppliers automatically; no further calls needed here
	}

	private boolean inLeftNeutralZone() {
		return rc.inLeftNeutralZone();
	}

	private boolean inRightNeutralZone() {
		return rc.inRightNeutralZone();
	}

	private boolean inScoringZone() {
		return rc.inScoringZone();
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
		if (inScoringZone()) {
			return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
		} else if (inLeftNeutralZone()) {
			// Aim past the tower toward our left scoring zone
			return AllianceFlipUtil.apply(
				FieldConstants.Tower.centerPoint.plus(new Translation2d(Units.metersToInches(-2.5), 0.0))
			);
		} else if (inRightNeutralZone()) {
			// Aim past the tower toward our right scoring zone
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
			ShootingOnTheMoveConstants.RPMOverapproxFactor; // m/s, this needed an efficiency factor

		double dx = target.getX() - turretPose.getTranslation().getX();
		double dy = target.getY() - turretPose.getTranslation().getY();
		double horizontalDist = Math.sqrt(dx * dx + dy * dy);

		// first-pass flight time approximation
		double estimatedFlightTime = horizontalDist / v0; // initial rough estimate
		double hoodSolution = Double.NaN;
		double adx = dx,
			ady = dy;

		// iteratively estimate flight time.
		// by default, aimIterations is set to 2. Must be >= 1.
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
		turretAngle = new Translation2d(adx, ady).getAngle().minus(robotPose.getRotation());
	}
}
