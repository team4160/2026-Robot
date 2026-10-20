package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShotingOnTheFlyConstants {

	// Robot center -> turret axis of rotation. +X = code front (drive motors 1 & 3), +Y = left.
	// Matches TurretSubsystem's MechanismPositionConfig. TODO: confirm with a tape measure.
	public static final Transform3d robotToTurret = new Transform3d(
		Inches.of(-4.5).in(Meters),
		Inches.of(-4.5).in(Meters),
		Inches.of(16.945).in(Meters),
		new Rotation3d(0, 0, Math.PI)
	);

	public static final double loopPeriodSecs = 0.02;

	/**
	 * Turret raw angle (deg) = robot-relative aim angle (deg) - this + trim. Raw 0 points out the front of the robot
	 * (180 aimed exactly opposite the hub on the first test).
	 */
	public static final double kTurretZeroOffsetDeg = 0.0;

	/**
	 * Small aim correction (deg). Live-tune with "SOTM/TurretTrimDeg" on the dashboard, then copy the value here so it
	 * survives a reboot.
	 */
	public static final double kTurretTrimDeg = 0.0;

	/**
	 * True = turret positive is opposite to robot-relative positive (the "-(...)" flip). Live-test with
	 * "SOTM/TurretFlipped" on the dashboard; if the other setting aims better, change this to match.
	 */
	public static final boolean kTurretFlipped = true;

	/** Turret usable range in raw degrees; kept a little inside the soft limits in TurretSubsystem (-80, 111). */
	public static final double kTurretMinDeg = -78.0;
	public static final double kTurretMaxDeg = 109.0;

	/** How far ahead to predict the robot pose to cover sensor/control latency (seconds). */
	public static final double kPhaseDelaySecs = 0.03;

	/**
	 * Air drag on the lead (from 6328's LaunchCalculator): the ball loses the robot's speed in flight, so we lead with
	 * effectiveTOF = (1 - e^(-tof * k)) / k instead of the real time of flight. 0.375 is 6328's value. 0 = off.
	 * Live-tune with "SOTM/DragConstant": moving shots land AHEAD of travel -> lower it, BEHIND -> raise it.
	 */
	public static final double kLinearDragTimeConstant = 0.375;

	/** Operator D-pad up/down nudges the flywheel by this much (RPM). Total shown as "SOTM/FlywheelOffsetRPM". */
	public static final double kFlywheelNudgeRPM = 50.0;

	/** Fixed-point iterations used to solve for time of flight while moving. */
	public static final int kLookaheadIterations = 10;

	/**
	 * Slow the drive while SOTM is tracking AND the operator holds right trigger (feeding). Toggle live with
	 * "SOTM/SlowDriveEnabled" on the dashboard.
	 */
	public static final boolean kSlowDriveEnabled = true;
	public static final double kSlowDriveTranslationScale = 0.5;
	public static final double kSlowDriveRotationScale = 0.6;

	// Readiness tolerances
	/**
	 * Turret is on target if the ball would still land within this many meters of the target center sideways, so the
	 * allowed angle = atan(radius / distance): looser up close, tighter far away. Hub opening is ~0.53 m radius; 0.35
	 * leaves room for the ball and other errors. Passing lands on the floor, so it can be much looser.
	 */
	public static final double kHubAimRadiusM = 0.35;
	public static final double kPassAimRadiusM = 1.0;
	/** Limits on that angle (deg) so it never gets silly at very short or very long range. */
	public static final double kTurretToleranceMinDeg = 3.0;
	public static final double kTurretToleranceMaxDeg = 10.0;
	public static final double kHoodToleranceDeg = 2.5;
	public static final double kShooterToleranceRPM = 250.0;

	/**
	 * Point the turret this far into the future (s) to make up for turret lag while moving. 0 = off. Tune 0.05-0.15
	 * live with "SOTM/TurretLeadSecs" (compare SOTM/TurretAimDeg vs TurretActualDeg), then copy the value here.
	 */
	public static final double kTurretLeadSecs = 0.08;

	/**
	 * Passing: in the neutral zone, lob FUEL back into our alliance zone instead of shooting at the hub. Switches
	 * automatically by robot position. Toggle live with "SOTM/PassEnabled".
	 */
	public static final boolean kPassEnabled = true;

	/** Start passing when the robot center is this far past our alliance zone line (m), about half the robot. */
	public static final double kPassStartOffsetM = 0.5;

	/** Dead band around that line so the mode doesn't flicker (m). */
	public static final double kPassHysteresisM = 0.15;

	/** Pass landing spots (blue coordinates, m), behind the bumps so the ball misses the hub. Uses the robot's side. */
	public static final Translation2d kPassTargetRight = new Translation2d(2.5, 2.5);
	public static final Translation2d kPassTargetLeft = new Translation2d(2.5, 5.57);

	/**
	 * Pass table: {distance from turret to pass spot (m), flywheel RPM, hood angle (deg), time of flight (s)}.
	 *
	 * <p>ESTIMATED from the measured shot table (2026-10-19), not measured. Fit from the 10-15 hub shots: ball speed
	 * ~1.765 m/s per 1000 RPM, launch angle ~ 77.6 - 0.56 * hood (deg), exit ~0.55 m high. Solved for landing on the
	 * floor at the pass spot with hood 30 (~61 deg launch; hood only measured 20-25, so the angle is extrapolated),
	 * +1% RPM per meter past 4 m for drag. Tune in TuningMode from the neutral zone (read SOTM/Distance); if passes
	 * land short/long at every distance, nudge all RPMs together. Keep rows sorted and hood within 1-35 deg.
	 */
	public static final double[][] kPassTable = {
		{ 2.0, 2550, 30.0, 0.92 }, // estimate
		{ 2.5, 2850, 30.0, 1.01 }, // estimate
		{ 3.0, 3150, 30.0, 1.10 }, // estimate
		{ 4.0, 3700, 30.0, 1.25 }, // estimate
		{ 5.0, 4200, 30.0, 1.39 }, // estimate
		{ 6.0, 4700, 30.0, 1.52 }, // estimate
		{ 7.0, 5150, 30.0, 1.63 }, // estimate
	};

	/**
	 * Shot table: {distance from turret to hub center (m), flywheel RPM, hood angle (deg), time of flight (s)}.
	 *
	 * <p>Measured standing still in tuning mode. Time of flight = 240 fps slow-mo playback time (Premiere sec:frames,
	 * 30 fps, or iPhone Photos seconds) / 8, ball leaves shooter -> drops through hub top. Whole flight must be inside the slow part.
	 * Distances from the left side of the field may be off (vision was less accurate there). Keep rows sorted by
	 * distance and hood within 1-35 deg. Range is 2.0-4.81 m; outside that SOTM/InRange is false.
	 */
	public static final double[][] kShotTable = {
		{ 2.00, 3200, 5.0, 1.01 }, // 10-08, OLD (before camera fix), re-tune
		{ 2.33, 3600, 10.0, 1.17 }, // 10-08, OLD (before camera fix), re-tune
		{ 2.87, 3500, 15.0, 1.03 }, // 10-08, OLD (before camera fix), re-tune
		{ 3.05, 3670, 20.0, 1.06 }, // 10-15
		{ 3.39, 4000, 23.0, 1.11 }, // 10-15
		{ 3.84, 4500, 24.0, 1.21 }, // 10-15
		{ 4.56, 4700, 24.0, 1.28 }, // 10-15
		{ 4.81, 4700, 25.0, 1.26 }, // 10-15
	};
}
