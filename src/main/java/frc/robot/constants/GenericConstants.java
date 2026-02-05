package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class GenericConstants {

	public static boolean disableHAL = false;

	public static void disableHAL() {
		disableHAL = true;
	}

	public static enum AimPoints {
		RED_HUB(new Translation3d(11.938, 4.034536, 1.5748)),
		RED_OUTPOST(new Translation3d(15.75, 7.25, 0)),
		RED_FAR_SIDE(new Translation3d(15.75, 0.75, 0)),

		BLUE_HUB(new Translation3d(4.5974, 4.034536, 1.5748)),
		BLUE_OUTPOST(new Translation3d(0.75, 0.75, 0)),
		BLUE_FAR_SIDE(new Translation3d(0.75, 7.25, 0));

		public final Translation3d value;

		private AimPoints(Translation3d value) {
			this.value = value;
		}

		public static final Translation3d getAllianceHubPosition() {
			return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_HUB.value : BLUE_HUB.value;
		}

		public static final Translation3d getAllianceOutpostPosition() {
			return DriverStation.getAlliance().get() == DriverStation.Alliance.Red
				? RED_OUTPOST.value
				: BLUE_OUTPOST.value;
		}

		public static final Translation3d getAllianceFarSidePosition() {
			return DriverStation.getAlliance().get() == DriverStation.Alliance.Red
				? RED_FAR_SIDE.value
				: BLUE_FAR_SIDE.value;
		}
	}
}
