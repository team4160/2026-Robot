// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.DrivebaseConstants;

public class Robot extends TimedRobot {

	private Command autonomousCommand;
	private Timer disabledTimer;

	public final RobotContainer robotContainer;

	public Robot() {
		robotContainer = new RobotContainer();
	}

	@Override
	public void robotInit() {
		// Create a timer to disable motor brake a few seconds after disable.
		// This will let the robot stop immediately when disabled, but then also let it be pushed more
		disabledTimer = new Timer();

		if (isSimulation()) {
			DriverStation.silenceJoystickConnectionWarning(true);
		}
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		robotContainer.setMotorBrake(true);
		disabledTimer.restart();
	}

	@Override
	public void disabledPeriodic() {
		if (disabledTimer.hasElapsed(DrivebaseConstants.WHEEL_LOCK_TIME)) {
			robotContainer.setMotorBrake(false);
			disabledTimer.stop();
			disabledTimer.reset();
		}
	}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		robotContainer.setMotorBrake(true);
		autonomousCommand = robotContainer.getAutonomousCommand();

		System.out.println("Auto selected: " + autonomousCommand);

		if (autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(autonomousCommand);
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		} else {
			CommandScheduler.getInstance().cancelAll();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
