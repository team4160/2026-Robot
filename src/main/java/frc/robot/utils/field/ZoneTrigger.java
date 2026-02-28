package frc.robot.utils.field;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ShotingOnTheFlyConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({ GeomUtil.class })
/** Zone Helper class to handle flipping n such. */
public class ZoneTrigger {

	/** The list of rectangle zones. */
	private final List<Rectangle2D> rectangles;

	/** The list of initial translation zones. */
	private final List<Pair<Translation2d, Translation2d>> zones;

	/** The pose supplier used to check whether the pose is inside the zone. */
	@Getter
	@Setter
	private Supplier<Pose2d> poseSupplier;

	/**
	 * A trigger that reads whether the robot is within the zone. Robot pose is supplied from the
	 * SwerveState.
	 */
	@Getter
	@Setter
	private Trigger trigger;

	/** The zone publisher. Advantage Scope trajectory shows a zone outline on the field. */
	@Getter
	@Setter
	private StructArrayPublisher<Translation2d> zonePublisher;

	/** The zone trigger, telling when the robot is in the zone. */
	@Getter
	@Setter
	private BooleanPublisher triggerPublisher;

	/** The network table telling where to publish the zone. */
	@Getter
	@Setter
	private NetworkTable zoneTable;

	/**
	 * The network table telling where to publish the zone. Typically defaults to a subtable of
	 * "Triggers" where the zone is published.
	 */
	@Getter
	@Setter
	private NetworkTable triggerTable;

	/**
	 * Creates a new auto flipping zone trigger.
	 *
	 * @param zones all the zone bounding boxes. The first is bottom left, Second is bottom right.
	 *     Each box is checked separately.
	 */
	@SafeVarargs
	public ZoneTrigger(String zoneName, Pair<Translation2d, Translation2d>... zones) {
		// Setup our zones.
		this.rectangles = new ArrayList<>();
		this.zones = List.of(zones);
		// Load the latest rectangles at construction
		loadRectangles();

		// Sets the publishing path to the telemetry table in our Telemetry class.
		/// This should be changed on differing projects.
		// this.setBothTables(Telemetry.Publishers.Robot.zoneTable);
		this.reloadPublishers(zoneName); // Reloads the publishers with our new tables.
		this.setPoseSupplier(() ->
			SwerveSubsystem.SwerveState.CurrentPose.transformBy(ShotingOnTheFlyConstants.robotToTurret.toTransform2d())
		);

		// Sets the trigger telling when our robot is within the zone.
		this.trigger = new Trigger(() -> containsPose(this.getPoseSupplier()));
		// Update the published trigger whenever the robot is enabled.
		RobotModeTriggers.teleop().whileTrue(Commands.run(this::publishTrigger));
		RobotModeTriggers.teleop().onTrue(Commands.runOnce(this::publishZone));
	}

	/**
	 * Sets the pose supplier, updating the default trigger in the process.
	 *
	 * @param supplier the pose supplier to check.
	 * @return this, for chaining.
	 */
	public ZoneTrigger withPoseSupplier(Supplier<Pose2d> supplier) {
		this.setPoseSupplier(supplier); // Update our pose supplier
		return this;
	}

	/**
	 * Checks if the given pose is held within the zones.
	 *
	 * @param pose the position to check.
	 * @return whether the pose is in the defined zone.
	 */
	public boolean containsPose(Pose2d pose) {
		// Checks to see if any rectangle contains the pose.
		return rectangles.stream().anyMatch(rect -> rect.contains(pose.getX(), pose.getY()));
	}

	/**
	 * Checks if the given pose is held within the zones.
	 *
	 * @param poseSupplier the position to check.
	 * @return whether the pose is in the defined zone.
	 */
	public boolean containsPose(Supplier<Pose2d> poseSupplier) {
		return containsPose(poseSupplier.get());
	}

	/**
	 * Flips the pose list if the robot is red alliance.
	 *
	 * @param zones the given zones.
	 * @return our zones at their expected side.
	 */
	private List<Pair<Translation2d, Translation2d>> ifShouldFlip(List<Pair<Translation2d, Translation2d>> zones) {
		System.out.println(DriverStation.getAlliance().toString());
		// If we should flip
		if (AllianceFlipUtil.shouldFlip()) {
			// Flip the zone
			final List<Pair<Translation2d, Translation2d>> updatedZones = new ArrayList<>();
			// Load a new pair for all current pairs.
			zones.forEach(pair -> {
				// Load our updated list with the first and second flipped to properly flip the width
				// and heights as well.
				updatedZones.add(
					Pair.of(AllianceFlipUtil.apply(pair.getSecond()), AllianceFlipUtil.apply(pair.getFirst()))
				); // Flip corners as well. First becomes second.
			});
			return updatedZones;
		} else {
			// Else, don't.
			return zones;
		}
	}

	/**
	 * Updates the {@link Rectangle2D}s relative to the current Alliance. Alliance doesn't like being
	 * checked at runtime, so it needs to be checked slightly later.
	 */
	private void loadRectangles() {
		this.rectangles.clear(); // Clear any previous rectangles.
		// Makes a Rect2d from the start and end poses of every bounding box.
		final List<Pair<Translation2d, Translation2d>> list = ifShouldFlip(zones);
		list.forEach(zone ->
			this.rectangles.add(
				new Rectangle2D.Double(
					// Get the initial bottom left position.
					zone.getFirst().getX(),
					zone.getFirst().getY(),
					// Width is the difference between the second and first X positions.
					zone.getSecond().getX() - zone.getFirst().getX(),
					// Height is the difference between the second and first Y positions.
					zone.getSecond().getY() - zone.getFirst().getY()
				)
			)
		);
	}

	/**
	 * Sets both the Zone and Trigger publishers with the same network table. The trigger will be in a
	 * "Triggers" subTable.
	 *
	 * @param table pointing to where things should be published.
	 * @return this, for chaining.
	 */
	public ZoneTrigger setBothTables(NetworkTable table) {
		// Set the zone table with the given path.
		this.setZoneTable(table);
		// Set the trigger table in a "Triggers" sub table.
		this.setTriggerTable(table.getSubTable("Triggers"));
		return this;
	}

	/**
	 * Sets both the Zone and Trigger publishers with the same network table path. The trigger will be
	 * in a "Triggers" subTable. For Example, Smartdashboard/RobotTelemetry/ZonePublishers
	 *
	 * @param path pointing to where things should be published.
	 * @return this, for chaining.
	 */
	public ZoneTrigger setBothTables(String path) {
		// Just calls setBothTables(NetworkTable) using the given path.
		return this.setBothTables(NetworkTableInstance.getDefault().getTable(path));
	}

	/**
	 * Reloads the publishers based on the given name and current tables.
	 *
	 * @param zoneName the name of the zone.
	 * @return this, for chaining.
	 */
	public ZoneTrigger reloadPublishers(String zoneName) {
		// Set the zone publisher.
		this.setZonePublisher(zoneTable.getStructArrayTopic(zoneName, Translation2d.struct).publish());
		// Set the trigger publisher
		this.setTriggerPublisher(triggerTable.getBooleanTopic(zoneName + " Trigger").publish());
		return this;
	}

	/** Publishes the latest trigger state. */
	private void publishTrigger() {
		this.getTriggerPublisher().accept(this.getTrigger().getAsBoolean());
	}

	/** Publishes a Translation in every bounding box corner. */
	private void publishZone() {
		loadRectangles(); // Update our rectangles first.
		final ArrayList<Translation2d> conePoses = new ArrayList<>(); // new array to store our points.
		// Load poses for every fresh rectangle.
		rectangles.forEach(zone -> {
			// The bottom left point.
			conePoses.add(new Translation2d(zone.getMinX(), zone.getMinY()));
			// The bottom right point.
			conePoses.add(new Translation2d(zone.getMaxX(), zone.getMinY()));
			// The top right point.
			conePoses.add(new Translation2d(zone.getMaxX(), zone.getMaxY()));
			// The top left point.
			conePoses.add(new Translation2d(zone.getMinX(), zone.getMaxY()));
			// Back to the bottom left point to close the outline.
			conePoses.add(new Translation2d(zone.getMinX(), zone.getMinY()));
		});
		// Publish our array.
		this.getZonePublisher().set(conePoses.toArray(Translation2d[]::new));
	}
}
