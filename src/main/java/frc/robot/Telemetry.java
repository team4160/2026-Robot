package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import lombok.Getter;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import swervelib.telemetry.SwerveDriveTelemetry;
import yams.motorcontrollers.SmartMotorControllerConfig;

public class Telemetry {

	/// Current Telemetry Setting
	public static TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.HIGH;
	/// Telemetry Defaults
	public static final String telemetryPath = "SmartDashboard/Telemetry"; // Make access public for other telemetry.
	public static final String smartDashboardPath = "Telemetry/";
	public static final NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable(telemetryPath);

	/// Publishers
	public static class Publishers {

		/// Robot Publishers
		public static class Robot {

			/// Telemetry Paths
			private static final NetworkTable robotTable = telemetryTable.getSubTable("RobotTelemetry");
			// Zones
			public static final NetworkTable zoneTable = robotTable.getSubTable("Zones");
			private static final String smartDashboardRobotPath = smartDashboardPath + "RobotTelemetry/";
			// // Input Selection
			// public static final SmartDashboardPublisher inputPublisher =
			//     new SmartDashboardPublisher(smartDashboardRobotPath + "Input Selector");
			// public static final StringPublisher inputOverride =
			//     robotTable.getSubTable("Input Selector").getStringTopic("selected").publish();

			// /// Mech3d Publishers
			// public static class Mech3D {
			//   public static final NetworkTable mechTable = robotTable.getSubTable("Mech3DTelemetry");
			//   public static final StructPublisher<Pose3d> hoodPose =
			//       mechTable.getStructTopic("Hood Pose", Pose3d.struct).publish();
			//   public static final StructPublisher<Pose3d> turretPose =
			//       mechTable.getStructTopic("Turret Pose", Pose3d.struct).publish();
			// }
		}

		// /// MapleSim Publishers
		// public static class MapleSim {
		//   // Table for maple sim publishers.
		//   private static final NetworkTable mapleTable =
		//       NetworkTableInstance.getDefault().getTable("SmartDashboard/MapleSim");
		//   // Generic Game Piece Publisher.
		//   public static final StructArrayPublisher<Pose3d> elementPublisher =
		//       mapleTable.getStructArrayTopic("Fuel", Pose3d.struct).publish();
		// }
	}

	/// Initializes any need data. Called statically.
	Telemetry() {
		// Add all the default pieces.
		if (RobotBase.isSimulation()) {
			((Arena2026Rebuilt) SimulatedArena.getInstance()).setEfficiencyMode(true); // Spawn more or less.
			SimulatedArena.getInstance().resetFieldForAuto(); // Reset the field.
		}
	}

	/// Updates all of our custom telemetry
	public static void updateTelemetry() {
		// // Input
		// Publishers.Robot.inputPublisher.update();

		if (RobotBase.isSimulation()) {
			// // MapleSim
			// Publishers.MapleSim.elementPublisher.accept(
			//     SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
		}
	}

	/** Quick Helper since RobotModeTriggers only has a few triggers. */
	public static class ModeTriggers {

		private static final Trigger isEnabledTrigger = new Trigger(DriverStation::isEnabled);

		public static Trigger isEnabled() {
			return isEnabledTrigger;
		}
	}

	/** SmartDashboard wrapper to match NT4 Publishers. */
	public static class SmartDashboardPublisher {

		/** The value published to the NT4. */
		@Getter
		private Sendable value;

		private Supplier<Sendable> supplier;
		private final String path;

		/**
		 * Small SmartDashboard Publisher Wrapper.
		 *
		 * @param path the telemetry path. Use smartDashboardPath + name.
		 */
		public SmartDashboardPublisher(String path) {
			this.path = path;
		}

		/**
		 * Puts the value onto the dashboard.
		 *
		 * @param value to publish.
		 */
		public void setValue(Sendable value) {
			this.value = value;
			SmartDashboard.putData(path, value);
		}

		/**
		 * Accepts sendable to be updated later. Also sets when ran.
		 *
		 * @param supplier the value supplier.
		 */
		public void accept(Supplier<Sendable> supplier) {
			this.supplier = supplier;
			setValue(supplier.get());
		}

		/** Updates the published value from the current supplier. */
		public void update() {
			setValue(supplier.get());
		}
	}

	/// Telemetry Verbosity Settings
	public enum TelemetryVerbosity {
		/// No telemetry data is sent to the dashboard.
		NONE(SmartMotorControllerConfig.TelemetryVerbosity.LOW, SwerveDriveTelemetry.TelemetryVerbosity.NONE),
		/// Only basic telemetry data is sent to the dashboard.
		LOW(SmartMotorControllerConfig.TelemetryVerbosity.LOW, SwerveDriveTelemetry.TelemetryVerbosity.LOW),
		/// All telemetry data is sent to the dashboard.
		HIGH(SmartMotorControllerConfig.TelemetryVerbosity.HIGH, SwerveDriveTelemetry.TelemetryVerbosity.HIGH);

		// Telemetry verbosity for YAMS at this verbosity level.
		public final SmartMotorControllerConfig.TelemetryVerbosity yamsVerbosity;
		// Telemetry verbosity for YAGSL at this verbosity level.
		public final SwerveDriveTelemetry.TelemetryVerbosity yagslVerbosity;

		/**
		 * Robot Telemetry Options
		 *
		 * @param yamsVerbosity Verbosity to use for YAMS at this level.
		 */
		TelemetryVerbosity(
			SmartMotorControllerConfig.TelemetryVerbosity yamsVerbosity,
			SwerveDriveTelemetry.TelemetryVerbosity yagslVerbosity
		) {
			this.yamsVerbosity = yamsVerbosity;
			this.yagslVerbosity = yagslVerbosity;
		}
	}
}
