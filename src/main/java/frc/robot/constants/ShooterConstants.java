package frc.robot.constants;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {

	public static final int kHood_ID = 51;
	public static final int kShooterLeader_ID = 61;
	public static final int kShooterFollower_ID = 62;

	public static final AngularVelocity kShooterVelocity = RPM.of(6000);

	public static final Double EncoderAOffset = 0.832406;
}
