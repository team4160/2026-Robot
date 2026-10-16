package frc.robot.constants;

/** Settings for the PathPlanner autos (named commands registered in RobotContainer). */
public class AutoConstants {

	// Intake arm angles: 0 = down (intaking), 92 = up (starting configuration). The arm can only drop right now.
	public static final double kArmDownDeg = 0.0;
	public static final double kArmUpDeg = 92.0;
	public static final double kArmToleranceDeg = 4.0;
	public static final double kArmMoveTimeoutSecs = 1.5;

	public static final double kIntakeDutyCycle = -0.75;

	// Feeding while shooting (same speeds as the "shoot" named command)
	public static final double kSpindexerFeedDutyCycle = 0.65;
	public static final double kKickerFeedDutyCycle = -1.0;

	/** How long to feed when shooting in auto. TODO: time how long it takes to empty the robot. */
	public static final double kFeedSecs = 2.5;

	/** Max time to wait for SOTM to be ready before feeding anyway. */
	public static final double kReadyTimeoutSecs = 1.5;

	// Jiggle: drive forward/back a little while feeding so balls settle into the spindexer
	public static final double kJiggleSpeedMps = 0.6;
	public static final double kJiggleHalfPeriodSecs = 0.15;
}
