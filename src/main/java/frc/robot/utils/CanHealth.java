package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Shows which CTRE devices are talking on the CAN bus, to find a broken spot in the CAN chain. Devices AFTER a break
 * all drop out together, so the break is just before the first missing device in the physical wiring order.
 *
 * <p>Dashboard: CAN/AllOK, CAN/Missing (names), CAN/Utilization %, error counters, and one light per device. Uses
 * Phoenix's isConnected() (cached, no extra CAN traffic). Extra device objects for the same IDs are fine in Phoenix 6.
 */
public class CanHealth extends SubsystemBase {

	private final Map<String, ParentDevice> devices = new LinkedHashMap<>();
	private double lastUpdate = 0;
	private String lastMissing = "";

	public CanHealth() {
		// Swerve (from deploy/swerve/kraken/modules/*.json)
		talon("FL Drive", 1);
		talon("FL Steer", 2);
		cancoder("FL Encoder", 1);
		talon("FR Drive", 3);
		talon("FR Steer", 4);
		cancoder("FR Encoder", 2);
		talon("BL Drive", 5);
		talon("BL Steer", 6);
		cancoder("BL Encoder", 3);
		talon("BR Drive", 7);
		talon("BR Steer", 8);
		cancoder("BR Encoder", 4);
		// Mechanisms
		talon("Intake Arm 11", 11);
		talon("Intake Arm 12", 12);
		cancoder("Intake Arm Encoder", 11);
		talon("Intake", 13);
		talon("Spindexer", 21);
		talon("Kicker", 31);
		cancoder("Turret Encoder", 41);
		talon("Turret", 43);
		talon("Hood", 51);
		talon("Shooter 61", 61);
		talon("Shooter 62", 62);
	}

	private void talon(String name, int id) {
		devices.put(name, new TalonFX(id));
	}

	private void cancoder(String name, int id) {
		devices.put(name, new CANcoder(id));
	}

	@Override
	public void periodic() {
		double now = Timer.getFPGATimestamp();
		if (now - lastUpdate < 0.25) return;
		lastUpdate = now;

		List<String> missing = new ArrayList<>();
		for (var e : devices.entrySet()) {
			boolean ok = e.getValue().isConnected();
			SmartDashboard.putBoolean("CAN/" + e.getKey(), ok);
			if (!ok) missing.add(e.getKey());
		}
		String missingText = String.join(", ", missing);
		SmartDashboard.putBoolean("CAN/AllOK", missing.isEmpty());
		SmartDashboard.putString("CAN/Missing", missingText);
		if (!missingText.equals(lastMissing)) {
			// Also goes to the Driver Station console/log with a timestamp, so it can be found after a match
			System.out.println(
				"[CAN] missing at " + String.format("%.1f", now) + "s: " + (missing.isEmpty() ? "none" : missingText)
			);
			lastMissing = missingText;
		}

		CANStatus status = RobotController.getCANStatus();
		SmartDashboard.putNumber("CAN/Utilization", status.percentBusUtilization * 100.0);
		SmartDashboard.putNumber("CAN/BusOffCount", status.busOffCount);
		SmartDashboard.putNumber("CAN/TxFullCount", status.txFullCount);
		SmartDashboard.putNumber("CAN/ReceiveErrorCount", status.receiveErrorCount);
		SmartDashboard.putNumber("CAN/TransmitErrorCount", status.transmitErrorCount);
	}
}
