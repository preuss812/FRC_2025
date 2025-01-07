// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.RobotContainer;

public class AdjustNoteInShooterCommand extends Command {
  private final ShooterSubsystem m_ShooterSubsystem;
  private int remainingTicks = 0;

  /** Creates a new TakeInNoteCommand. */
  public AdjustNoteInShooterCommand(
    ShooterSubsystem shooterSubsystem
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.m_OpticalLimitSwitch.isClosed()) {
      remainingTicks = (int)(0.8*50);
      m_ShooterSubsystem.unshoot();
    } else {
      remainingTicks = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotContainer.m_OpticalLimitSwitch.isClosed())
      remainingTicks--;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_ShooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (remainingTicks <= 0);
  }
}
