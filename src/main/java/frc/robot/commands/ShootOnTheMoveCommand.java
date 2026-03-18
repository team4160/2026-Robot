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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private double hoodAngle = Double.NaN;  // Why is it NaN?
    private double turretVelocity;
    private double hoodVelocity;

    private LaunchingParameters latestParameters = null;

    private final ShootingProfile profile = new ShootingProfile();  // see bottom of script

    public record LaunchingParameters(
        boolean isValid,
        Rotation2d turretAngle,
        double turretVelocity,
        double hoodAngle,
        double hoodVelocity,
        double flywheelSpeed
    ) {}

	public ShootOnTheMoveCommand(SwerveSubsystem drivetrain, ScoringSystem superstructure) {
		this.drivetrain = drivetrain;
		this.superstructure = superstructure;
	}

	@Override
	public void initialize() {
		super.initialize();

        lastHoodAngle = superstructure.getHoodAngle().in(Degrees);
        lastTurretAngle = Rotation2d.fromRotations(superstructure.getTurretAngle().in(Degrees));
        lastShootSpeed = superstructure.getShooterSpeed();

        superstructure.aimDynamicCommand(
            () -> lastShootSpeed,
            () -> Rotations.of(lastTurretAngle.getRotations()),
            () -> Rotations.of(lastHoodAngle)
        ).schedule();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

    @Override
    public void aim() {
        aimHandler(turretPosition, target, chassisVel);
    }

	@Override
	public void execute() {
		Pose2d estimatedPose = estimatePoseWithPhaseDelay();

		// Designate desired target
		target = getCurrentTarget();

		// Calculate distance from turret to target
		Pose2d turretPosition = estimatedPose.transformBy(ShotingOnTheFlyConstants.robotToTurret.toTransform2d());
		double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());
		SmartDashboard.putNumber("Turret to Target Distance", turretToTargetDistance);

		// Calculate field relative turret velocity
		ChassisSpeeds chassisVel = drivetrain.getFieldVelocity();
		Translation2d turretVelocity = getPointVelocity(chassisVel, estimatedPose.getRotation().getRadians(), ShotingOnTheFlyConstants.robotToTurret.toTranslation2d());
		double turretVelocityX = turretVelocity.getX();
		double turretVelocityY = turretVelocity.getY();

		// Kinematic approach
		aimHandler(turretPosition, target, chassisVel);

		superstructure.setShooterSetpoints(
			RPM.of(launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance)),
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

    public void aimHandler(Pose2d turretPose, Translation2d target, ChassisSpeeds chassisVel) {
        if (inLeftNeutralZone() || inRightNeutralZone()) {
            hoardFuelInZoneAim(turretPosition, chassisVel);
            return;
        }
        
        // Kinematic approach
		kinematicLaunchingParameters(turretPosition, target, chassisVel, 2.4);
    }

    public void hoardFuelInZoneAim(Pose2d turretPose, ChassisSpeeds chassisVel) {
        double y = 3.0;
        if (turretPose.y < 0) {
            y *= -1.0;
        }
        Translation2d target = new Translation2d(-5.0, y);

        kinematicLaunchingParameters(turretPose, target, chassisVel, 0.2);
    }

	public void clearLaunchingParameters() {
		latestParameters = null;
	}

	private Pose2d estimatePoseWithPhaseDelay() {
        Pose2d pose = drivetrain.getPose();
        ChassisSpeeds vel = drivetrain.getRobotVelocity();
        return pose.exp(new Twist2d(
            vel.vxMetersPerSecond * profile.phaseDelay,
            vel.vyMetersPerSecond * profile.phaseDelay,
            vel.omegaRadiansPerSecond * profile.phaseDelay
        ));
    }

	private Translation2d getCurrentTarget() {
        if (ScoringSystem.CustomTriggers.scoringZone.getTrigger().getAsBoolean() ||
            ScoringSystem.CustomTriggers.bumpZone.getTrigger().getAsBoolean()) {
            return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        } 
		else if (inLeftNeutralZone()) {
            return AllianceFlipUtil.apply(FieldConstants.LeftBump.nearRightCorner.plus(new Translation2d(0, Inches.of(36.5).in(Meters))));
        } 
		else if (inRightNeutralZone()) {
            return AllianceFlipUtil.apply(FieldConstants.RightBump.nearRightCorner.plus(new Translation2d(0, Inches.of(36.5).in(Meters))));
        } 
		else {
            return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        }
    }

	public static Translation2d getPointVelocity(ChassisSpeeds chassisVelocity, double robotAngle, Translation2d pointOffset) {
		double vx = chassisVelocity.vxMetersPerSecond +
			chassisVelocity.omegaRadiansPerSecond * (pointOffset.getY() * Math.cos(robotAngle) - pointOffset.getX() * Math.sin(robotAngle));
		double vy = chassisVelocity.vyMetersPerSecond +
			chassisVelocity.omegaRadiansPerSecond * (pointOffset.getX() * Math.cos(robotAngle) - pointOffset.getY() * Math.sin(robotAngle));
		return new Translation2d(vx, vy);
	}

	// MY IDEA: Kinematics?
	// After watching some FRC footage, I noticed our shooting was kind of weak
	// I watched some clips from an FRC Istanbul regional final this year, and noticed their arc was forgivably parabolic.
	// Hopefully this helps.
	private void kinematicLaunchingParameters(Pose2d turretPose, Translation2d target, ChassisSpeeds chassisVel) {
		double dx = target.getX() - turretPose.getTranslation().getX();
		double dy = target.getY() - turretPose.getTranslation().getY();	

		double horizontalDist = Math.sqrt(dx*dx + dy*dy);

		double v0 = profile.getFlywheelSpeed(horizontalDist);  // m/s, maybe this needs an efficiency factor
		double estimatedFlightTime = horizontalDist / v0;

		// offset
		dx -= chassisVel.vxMetersPerSecond * estimatedFlightTime;
		dy -= chassisVel.vyMetersPerSecond * estimatedFlightTime;

		double adjustedDist = Math.sqrt(dx*dx + dy*dy);
		double verticalDist = 2.4;  // What is the height of the bin we shoot in (in meters)?

		// quadratic discriminant
		double g = 9.81;  // this should be elsewhere
		double v2 = v0 * v0;
		double discr = v2*v2 - g * (g*adjustedDist*adjustedDist + 2*verticalDist*v2);

		if (discr < 0) {
			// Imaginary solutions; cannot reach target
			turretAngle = Rotation2d.fromDegrees(0);  // fallback
			hoodAngle = 0;
		}
		else {
			// High arc (additive) solution
			hoodAngle = Math.atan((v2 + Math.sqrt(discr)) / (g * adjustedDist));
		}

		Pose2d robotPose = drivetrain.getPose();
		turretAngle = target.minus(turretPose.getTranslation()).getAngle().minus(robotPose.getRotation());

		// why is this necessary for every loop?  why not on init?
		if (lastTurretAngle == null) lastTurretAngle = turretAngle;
		if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

		// Should these be abstracted into two calls of one "calcFilteredDerivative" function?
		turretVelocity = turretAngleFilter.calculate((turretAngle.minus(lastTurretAngle)).getRadians() / ShotingOnTheFlyConstants.loopPeriodSecs);
		hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / ShotingOnTheFlyConstants.loopPeriodSecs);

		lastTurretAngle = turretAngle;
		lastHoodAngle = hoodAngle;

		// update launching params
		latestParameters = new LaunchingParameters(
			adjustedDist >= profile.minDistance && adjustedDist <= profile.maxDistance,
			turretAngle, turretVelocity,
			hoodAngle, hoodVelocity,
			v0
		);

		lastShootSpeed = v0;
	}

	// Shooting profile containing static maps and constants 
	// TODO: bring in from json instead of hardcoding here.
    private static class ShootingProfile {
        final double minDistance = 1.34;
        final double maxDistance = 5.60;
        final double phaseDelay = 0.03;

        final InterpolatingTreeMap<Double, Rotation2d> hoodMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
        final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
        final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

        ShootingProfile() {
            hoodMap.put(1.34, Rotation2d.fromDegrees(19.0));
            hoodMap.put(1.78, Rotation2d.fromDegrees(19.0));
            hoodMap.put(2.17, Rotation2d.fromDegrees(24.0));
            hoodMap.put(2.81, Rotation2d.fromDegrees(27.0));
            hoodMap.put(3.82, Rotation2d.fromDegrees(29.0));
            hoodMap.put(4.09, Rotation2d.fromDegrees(30.0));
            hoodMap.put(4.40, Rotation2d.fromDegrees(31.0));
            hoodMap.put(4.77, Rotation2d.fromDegrees(32.0));
            hoodMap.put(5.57, Rotation2d.fromDegrees(32.0));
            hoodMap.put(5.60, Rotation2d.fromDegrees(35.0));

            flywheelMap.put(Inches.of(118.51).in(Meters), 7500.0);
            flywheelMap.put(Inches.of(111.51).in(Meters), 7000.0);
            flywheelMap.put(Inches.of(97.51).in(Meters), 6800.0);
            flywheelMap.put(Inches.of(87.51).in(Meters), 6600.0);
            flywheelMap.put(Inches.of(77.51).in(Meters), 6100.0);

            timeOfFlightMap.put(5.68, 1.16);
            timeOfFlightMap.put(4.55, 1.12);
            timeOfFlightMap.put(3.15, 1.11);
            timeOfFlightMap.put(1.88, 1.09);
            timeOfFlightMap.put(1.38, 0.90);
        }

        Rotation2d getHoodAngle(double distance) {
            return hoodMap.get(distance);
        }

        double getFlywheelSpeed(double distance) {
            return flywheelMap.get(distance);
        }

        double getTimeOfFlight(double distance) {
            return timeOfFlightMap.get(distance);
        }
    }
}
