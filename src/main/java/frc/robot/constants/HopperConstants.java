package frc.robot.constants;

public class HopperConstants {

	public static final int kHopper1_ID = 11;
	public static final int kHopper2_ID = 12;
	public static final int kSpindexer_ID = 21;
	public static final int kKicker_ID = 31;

	/**
	 * Spindexer agitation (idea from 4414's "dye rotor"): when not feeding, spin slowly BACKWARDS to keep balls loose
	 * so they don't pack. Feeding is +0.65. Toggle live with "Hopper/AgitateEnabled".
	 */
	public static final boolean kAgitateEnabled = false; // off: our spindexer is UNDER the false floor, so it can't stir the pile like 4414's rotor
	public static final double kAgitateDutyCycle = -0.08;
}
