# 2026-Robot

This repository is a Java/WPILib FRC robot project.

## Safe way to experiment (without changing `main`)

Run these commands once in your local clone:

```bash
git switch main
git pull
git switch -c playground/<your-name>
```

Example:

```bash
git switch -c playground/alex
```

Now you can change anything you want on your playground branch. `main` stays unchanged unless you explicitly merge into it.

## Switching back and forth

```bash
git switch main               # go back to stable code
git switch playground/<name>  # return to your playground work
```

## Project layout (quick map)

- `src/main/java/frc/robot/Robot.java` – robot lifecycle entry points
- `src/main/java/frc/robot/RobotContainer.java` – command/controller wiring
- `src/main/java/frc/robot/subsystems` – subsystem implementations
- `src/main/java/frc/robot/commands` – robot commands
- `src/main/java/frc/robot/constants` – configuration constants

## If you want to try Python

WPILib supports RobotPy, but this repo is currently configured for Java (`build.gradle`).
A safe approach is to experiment with Python in your `playground/*` branch first, then decide later if you want to migrate.
