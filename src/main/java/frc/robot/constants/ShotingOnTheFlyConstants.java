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

	/** Turret usable range in raw degrees; kept a little inside the soft limits in TurretSubsystem (-80, 111). */
	public static final double kTurretMinDeg = -78.0;
	public static final double kTurretMaxDeg = 109.0;

	/** How far ahead to predict the robot pose to cover sensor/control latency (seconds). */
	public static final double kPhaseDelaySecs = 0.03;

	/** Fixed-point iterations used to solve for time of flight while moving. */
	public static final int kLookaheadIterations = 10;

	// Readiness tolerances
	public static final double kTurretToleranceDeg = 2.0;
	public static final double kHoodToleranceDeg = 1.0;
	public static final double kShooterToleranceRPM = 100.0;

	/**
	 * Shot table: {distance from turret to hub center (m), flywheel RPM, hood angle (deg), time of flight (s)}.
	 *
	 * <p>RPM + hood measured stationary on 2026-10-06 (tuning mode). Only covers 3.0-4.6 m; outside that SOTM/InRange is
	 * false. Time of flight from 240 fps slow-mo (ball leaves shooter -> drops through hub top), assuming a 30 fps
	 * Premiere sequence: slow-mo playback time / 8. Check point: 3.87 m measured 1.27 s. TODO: add rows near 3.3 m and
	 * 4.2 m, hood jumps a lot between 3.0 and 3.7 m. Keep rows sorted by distance and hood within 1-35 deg.
	 */
	public static final double[][] kShotTable = {
		{ 3.02, 4000, 12.5, 1.19 }, // avg of 3.01 m / hood 13 and 3.04 m / hood 12; tof from video 1 (9:15 slow-mo)
		{ 3.68, 4200, 25.0, 1.25 }, // tof estimated between video 1 and the 3.87 m check point (no video)
		{ 4.64, 4500, 26.0, 1.29 }, // tof from video 2 (10:10 slow-mo)
	};
}
