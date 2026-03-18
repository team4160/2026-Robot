package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class ShootingOnTheMoveConstants {

	public static final Transform3d robotToTurret = new Transform3d(
		Inches.of(-5.25).in(Meters),
		Inches.of(5.25).in(Meters),
		Inches.of(16.945).in(Meters),
		new Rotation3d(0, 0, 180)
	);

	public static final double phaseDelay   = 0.03;
	public static final double flywheelRPM  = 2000.0;
	public static final double flywheelNeutralZoneRPM = 1000.0;
	public static final double flywheelRadiusMeters = Inches.of(2).in(Meters);
	public static final double gAcceleration = 9.81;
	public static final int aimIterations = 2;  // must be >= 1; for accurate shooting on the move. 
}
