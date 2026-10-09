package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

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

	/** Fixed-point iterations used to solve for time of flight while moving. */
	public static final int kLookaheadIterations = 10;

	// Readiness tolerances
	public static final double kTurretToleranceDeg = 3.0;
	public static final double kHoodToleranceDeg = 1.5;
	public static final double kShooterToleranceRPM = 150.0;

	/**
	 * Shot table: {distance from turret to hub center (m), flywheel RPM, hood angle (deg), time of flight (s)}.
	 *
	 * <p>Measured standing still in tuning mode. Time of flight = 240 fps slow-mo playback time (Premiere sec:frames,
	 * 30 fps) / 8, ball leaves shooter -> drops through hub top. 2026-10-08 session replaces the old 3.02 m row.
	 * Distances from the left side of the field may be off (vision was less accurate there). Keep rows sorted by
	 * distance and hood within 1-35 deg. Range is 2.0-4.9 m; outside that SOTM/InRange is false.
	 */
	public static final double[][] kShotTable = {
		{ 2.00, 3200, 5.0, 1.01 }, // 10-08, 8:02
		{ 2.33, 3600, 10.0, 1.17 }, // 10-08, 9:10 (higher arc than neighbors, so longer tof)
		{ 2.87, 3500, 15.0, 1.03 }, // 10-08, 8:06
		{ 3.23, 4100, 21.0, 1.06 }, // 10-08, 8:15
		{ 3.68, 4200, 25.0, 1.25 }, // 10-06, tof estimated
		{ 4.64, 4500, 26.0, 1.29 }, // 10-06, 10:10
		{ 4.91, 4300, 20.0, 1.31 }, // 10-08, 10:15 (trim was -27 on this shot, double-check)
	};
}
