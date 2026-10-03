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
	 * <p>TODO: THESE ARE PLACEHOLDERS - tune on the real robot. Turn on "SOTM/TuningMode", park the robot at
	 * several distances, dial in RPM + hood on the dashboard until it scores, and record "SOTM/Distance". Time of
	 * flight is best measured from slow-mo video (ball leaves shooter -> ball enters hub). Hood must stay within
	 * 1-35 deg.
	 */
	public static final double[][] kShotTable = {
		{ 1.5, 3400, 12.0, 0.80 },
		{ 2.0, 3600, 17.0, 0.85 },
		{ 2.5, 3800, 21.0, 0.90 },
		{ 3.0, 4000, 23.0, 0.95 },
		{ 3.5, 4200, 26.0, 1.00 },
		{ 4.0, 4400, 28.0, 1.05 },
		{ 4.5, 4650, 30.0, 1.10 },
		{ 5.0, 4900, 32.0, 1.15 },
		{ 5.5, 5150, 34.0, 1.20 },
	};
}
