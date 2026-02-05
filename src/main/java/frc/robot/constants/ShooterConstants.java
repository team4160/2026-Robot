package frc.robot.constants;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {

	public static int kHood_ID = 51;
	public static int kShooterLeader_ID = 61;
	public static int kShooterFollower_ID = 62;

	public static AngularVelocity kShooterVelocity = RPM.of(6000);

	public static Double EncoderAOffset = 0.832406;
}
