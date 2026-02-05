package frc.robot.systems;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.field.FieldConstants;

public class GameData {

	// private CoralPlacer          m_coralPlacer;
	// private Elevator             m_elevator;
	private SwerveSubsystem m_drivebase;

	// private TargetingSystem      m_targetSystem;
	// private CoralFunnel          m_coralFunnel;
	// private Climber              m_climber;

	public GameData(
		// CoralPlacer coralPlacer,
		// Elevator elevator,
		SwerveSubsystem drivebase
		// TargetingSystem targeting,
		// CoralFunnel coralFunnel,
		// Climber climber
	) {
		// m_coralPlacer = coralPlacer;
		// m_elevator = elevator;
		m_drivebase = drivebase;

		// m_targetSystem = targeting;
		// m_coralFunnel = coralFunnel;
		// m_climber = climber;
	}

	public void periodic() {
		SmartDashboard.putBoolean("can_shoot", canShoot());
	}

	public Trigger inScoringZone() {
		return new Trigger(() ->
			new Rectangle2d(
				new Translation2d(0, 0),
				new Translation2d(FieldConstants.LinesVertical.starting, FieldConstants.fieldWidth)
			).contains(m_drivebase.getPose().getTranslation())
		);
	}

	public boolean canShoot() {
		// String gameData;
		boolean canShootBool = true;
		DriverStation.Alliance alliance = DriverStation.getAlliance().get();
		boolean is_blue = (alliance == DriverStation.Alliance.Blue);
		String gameData = DriverStation.getGameSpecificMessage();
		double matchTime = DriverStation.getMatchTime();
		if (gameData.length() > 0) {
			switch (gameData.charAt(0)) {
				case 'B':
					// Blue case code
					if (is_blue && (matchTime >= 140 && matchTime <= 130)) {
						canShootBool = false;
					} else if (!is_blue && (matchTime >= 130 && matchTime <= 105)) {
						canShootBool = false;
					} else if (is_blue && (matchTime >= 105 && matchTime <= 55)) {
						canShootBool = false;
					} else if (!is_blue && (matchTime >= 55 && matchTime <= 30)) {
						canShootBool = false;
					}
					break;
				case 'R':
					if (!is_blue && (matchTime >= 140 && matchTime <= 130)) {
						canShootBool = false;
					} else if (is_blue && (matchTime >= 130 && matchTime <= 105)) {
						canShootBool = false;
					} else if (!is_blue && (matchTime >= 105 && matchTime <= 55)) {
						canShootBool = false;
					} else if (is_blue && (matchTime >= 55 && matchTime <= 30)) {
						canShootBool = false;
					}
				// Red case code
				default:
					// This is corrupt data
					canShootBool = true;
					break;
			}
		} else {
			// Code for no data received yet
			canShootBool = true;
		}
		return canShootBool;
	}
}
