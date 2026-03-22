// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.subsystems.Vision.Cameras;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Setter;
import org.json.simple.parser.ParseException;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

	/**
	 * Swerve drive object.
	 */
	private final SwerveDrive swerveDrive;

	/**
	 * Enable vision odometry updates while driving.
	 * Flip to true to enable PhotonVision pose correction.
	 */
	private boolean visionOdometer = true;

	/**
	 * Runtime kill switch for vision. Can be toggled via toggleVisionEnabled().
	 * Useful if vision is producing bad estimates mid-competition — disable
	 * without redeploying.
	 */
	private boolean visionEnabled = true;

	/**
	 * PhotonVision class to keep an accurate odometry.
	 */
	private Vision vision;

	public static class SwerveState {

		@Setter
		public static Supplier<SwerveDrive> swerveDrive = () -> null;

		@Setter
		public static Pose2d CurrentPose = Pose2d.kZero;

		@Setter
		public static ChassisSpeeds CurrentSpeeds = new ChassisSpeeds();
	}

	// overload
	public void toggleVisionEnabled() {
		visionEnabled = !visionEnabled;
	}

	public void toggleVisionEnabled(boolean v) {
		visionEnabled = v;
	}

	public boolean getVisionEnabled() {
		return visionEnabled;
	}

	/**
	 * Initialize {@link SwerveDrive} with the directory provided.
	 *
	 * @param directory Directory of swerve drive config files.
	 */
	public SwerveSubsystem(File directory) {
		boolean blueAlliance =
			DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue;
		Pose2d startingPose = blueAlliance
			? new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0))
			: new Pose2d(new Translation2d(Meter.of(16), Meter.of(4)), Rotation2d.fromDegrees(180));
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
		try {
			swerveDrive = new SwerveParser(directory).createSwerveDrive(DrivebaseConstants.MAX_SPEED, startingPose);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
		swerveDrive.setHeadingCorrection(false);
		if (!SwerveDriveTelemetry.isSimulation) swerveDrive.setCosineCompensator(false);
		swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
		swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
		if (visionOdometer) {
			setupPhotonVision();
			swerveDrive.stopOdometryThread();
		}
		setupPathPlanner();
		SwerveState.swerveDrive = () -> swerveDrive;
	}

	/**
	 * Construct the swerve drive.
	 *
	 * @param driveCfg      SwerveDriveConfiguration for the swerve.
	 * @param controllerCfg Swerve Controller.
	 */
	public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
		swerveDrive = new SwerveDrive(
			driveCfg,
			controllerCfg,
			DrivebaseConstants.MAX_SPEED,
			new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)), Rotation2d.fromDegrees(0))
		);
	}

	/**
	 * Setup the photon vision class.
	 */
	public void setupPhotonVision() {
		vision = new Vision(swerveDrive::getPose, swerveDrive.field);
	}

	@Override
	public void periodic() {
		if (visionOdometer) {
			swerveDrive.updateOdometry();
			if (visionEnabled) {
				vision.updatePoseEstimation(swerveDrive);
				vision.updateVisionField();
			}
		}
		SwerveState.CurrentPose = swerveDrive.getPose();
		SwerveState.CurrentSpeeds = swerveDrive.getRobotVelocity();
	}

	@Override
	public void simulationPeriodic() {}

	/**
	 * Setup AutoBuilder for PathPlanner.
	 */
	public void setupPathPlanner() {
		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();

			final boolean enableFeedforward = true;
			AutoBuilder.configure(
				this::getPose,
				this::resetOdometry,
				this::getRobotVelocity,
				(speedsRobotRelative, moduleFeedForwards) -> {
					if (enableFeedforward) {
						swerveDrive.drive(
							speedsRobotRelative,
							swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
							moduleFeedForwards.linearForces()
						);
					} else {
						swerveDrive.setChassisSpeeds(speedsRobotRelative);
					}
				},
				new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
				config,
				() -> {
					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this
			);
		} catch (Exception e) {
			e.printStackTrace();
		}

		PathfindingCommand.warmupCommand().schedule();
	}

	/**
	 * Aim the robot at the target returned by PhotonVision.
	 *
	 * @return A {@link Command} which will run the alignment.
	 */
	public Command aimAtTarget(Cameras camera) {
		return run(() -> {
			Optional<PhotonPipelineResult> resultO = camera.getBestResult();
			if (resultO.isPresent()) {
				var result = resultO.get();
				if (result.hasTargets()) {
					drive(getTargetSpeeds(0, 0, Rotation2d.fromDegrees(result.getBestTarget().getYaw())));
				}
			}
		});
	}

	public Command getAutonomousCommand(String pathName) {
		return new PathPlannerAuto(pathName);
	}

	public Command driveToPose(Pose2d pose) {
		PathConstraints constraints = new PathConstraints(
			swerveDrive.getMaximumChassisVelocity(),
			4.0,
			swerveDrive.getMaximumChassisAngularVelocity(),
			Units.degreesToRadians(720)
		);
		return AutoBuilder.pathfindToPose(pose, constraints, edu.wpi.first.units.Units.MetersPerSecond.of(0));
	}

	public Command driveToPose(Supplier<Pose2d> pose) {
		return defer(() -> driveToPose(pose.get()));
	}

	public Command driveToSetPoint(double x, double y, double angle) {
		return driveToPose(new Pose2d(new Translation2d(Meter.of(x), Meter.of(y)), Rotation2d.fromDegrees(angle)));
	}

	private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
		throws IOException, ParseException {
		SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
			RobotConfig.fromGUISettings(),
			swerveDrive.getMaximumChassisAngularVelocity()
		);
		AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
			new SwerveSetpoint(
				swerveDrive.getRobotVelocity(),
				swerveDrive.getStates(),
				DriveFeedforwards.zeros(swerveDrive.getModules().length)
			)
		);
		AtomicReference<Double> previousTime = new AtomicReference<>();

		return startRun(
			() -> previousTime.set(Timer.getFPGATimestamp()),
			() -> {
				double newTime = Timer.getFPGATimestamp();
				SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(
					prevSetpoint.get(),
					robotRelativeChassisSpeed.get(),
					newTime - previousTime.get()
				);
				swerveDrive.drive(
					newSetpoint.robotRelativeSpeeds(),
					newSetpoint.moduleStates(),
					newSetpoint.feedforwards().linearForces()
				);
				prevSetpoint.set(newSetpoint);
				previousTime.set(newTime);
			}
		);
	}

	public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
		try {
			return driveWithSetpointGenerator(() ->
				ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading())
			);
		} catch (Exception e) {
			DriverStation.reportError(e.toString(), true);
		}
		return Commands.none();
	}

	public Command sysIdDriveMotorCommand() {
		return SwerveDriveTest.generateSysIdCommand(
			SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true),
			3.0,
			5.0,
			3.0
		);
	}

	public Command sysIdAngleMotorCommand() {
		return SwerveDriveTest.generateSysIdCommand(
			SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive),
			3.0,
			5.0,
			3.0
		);
	}

	public Command centerModulesCommand() {
		return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
	}

	public Command driveForward() {
		return run(() -> swerveDrive.drive(new Translation2d(1, 0), 0, false, false)).finallyDo(() ->
			swerveDrive.drive(new Translation2d(0, 0), 0, false, false)
		);
	}

	public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
		swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
	}

	public Command driveCommand(
		DoubleSupplier translationX,
		DoubleSupplier translationY,
		DoubleSupplier angularRotationX
	) {
		return run(() ->
			swerveDrive.drive(
				SwerveMath.scaleTranslation(
					new Translation2d(
						translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
						translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
					),
					0.8
				),
				Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
				true,
				false
			)
		);
	}

	public Command driveCommand(
		DoubleSupplier translationX,
		DoubleSupplier translationY,
		DoubleSupplier headingX,
		DoubleSupplier headingY
	) {
		return run(() -> {
			Translation2d scaledInputs = SwerveMath.scaleTranslation(
				new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()),
				0.8
			);
			driveFieldOriented(
				swerveDrive.swerveController.getTargetSpeeds(
					scaledInputs.getX(),
					scaledInputs.getY(),
					headingX.getAsDouble(),
					headingY.getAsDouble(),
					swerveDrive.getOdometryHeading().getRadians(),
					swerveDrive.getMaximumChassisVelocity()
				)
			);
		});
	}

	public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
		swerveDrive.drive(translation, rotation, fieldRelative, false);
	}

	public void driveFieldOriented(ChassisSpeeds velocity) {
		swerveDrive.driveFieldOriented(velocity);
	}

	public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
		return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
	}

	public void drive(ChassisSpeeds velocity) {
		swerveDrive.drive(velocity);
	}

	public SwerveDriveKinematics getKinematics() {
		return swerveDrive.kinematics;
	}

	public void resetOdometry(Pose2d initialHolonomicPose) {
		swerveDrive.resetOdometry(initialHolonomicPose);
	}

	public Pose3d getPose3d() {
		return new Pose3d(swerveDrive.getPose());
	}

	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		swerveDrive.setChassisSpeeds(chassisSpeeds);
	}

	public void postTrajectory(Trajectory trajectory) {
		swerveDrive.postTrajectory(trajectory);
	}

	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	private boolean isRedAlliance() {
		var alliance = DriverStation.getAlliance();
		return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
	}

	/**
	 * Zero the gyro and set heading based on alliance.
	 * Red alliance faces 180°, Blue alliance faces 0°.
	 */
	public void zeroGyroWithAlliance() {
		if (isRedAlliance()) {
			zeroGyro();
			resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
		} else {
			zeroGyro();
		}
	}

	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	public Rotation2d getHeading() {
		return getPose().getRotation();
	}

	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
		Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
		return swerveDrive.swerveController.getTargetSpeeds(
			scaledInputs.getX(),
			scaledInputs.getY(),
			headingX,
			headingY,
			getHeading().getRadians(),
			DrivebaseConstants.MAX_SPEED
		);
	}

	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
		Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
		return swerveDrive.swerveController.getTargetSpeeds(
			scaledInputs.getX(),
			scaledInputs.getY(),
			angle.getRadians(),
			getHeading().getRadians(),
			DrivebaseConstants.MAX_SPEED
		);
	}

	public ChassisSpeeds getFieldVelocity() {
		return swerveDrive.getFieldVelocity();
	}

	public ChassisSpeeds getRobotVelocity() {
		return swerveDrive.getRobotVelocity();
	}

	public SwerveController getSwerveController() {
		return swerveDrive.swerveController;
	}

	public SwerveDriveConfiguration getSwerveDriveConfiguration() {
		return swerveDrive.swerveDriveConfiguration;
	}

	public void lock() {
		swerveDrive.lockPose();
	}

	public Rotation2d getPitch() {
		return swerveDrive.getPitch();
	}

	public void addFakeVisionReading() {
		swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
	}

	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}
}
