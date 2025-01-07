// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystemSRX;

// This command just starts the robot spinning and keeps rotating at the requested speed.
// That means that some other outside force has to end this command.
public class PushTowardsWall extends Command {
  private final DriveSubsystemSRX robotDrive;
  /** Creates a new RotateRobotCommand. */
  public PushTowardsWall(DriveSubsystemSRX robotDrive) {
    this.robotDrive = robotDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotDrive.drive(0, 0.1, 0, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.drive(0,0,0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
