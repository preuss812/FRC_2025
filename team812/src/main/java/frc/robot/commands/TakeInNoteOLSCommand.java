// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.RobotContainer;

public class TakeInNoteOLSCommand extends Command {
  private final NoteIntakeSubsystem m_NoteIntakeSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;

  /** Creates a new TakeInNoteCommand. */
  public TakeInNoteOLSCommand(
    NoteIntakeSubsystem noteIntakeSubsystem,
    ShooterSubsystem shooterSubsystem
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_NoteIntakeSubsystem = noteIntakeSubsystem;
    m_ShooterSubsystem = shooterSubsystem;
    addRequirements(noteIntakeSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_NoteIntakeSubsystem.pickUpNote();
    m_ShooterSubsystem.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_NoteIntakeSubsystem.stop();
      m_ShooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return m_colorDetectionSubsytem.isOrange(m_colorDetectionSubsytem.get_color());
    SmartDashboard.putBoolean("OpticalLimit", RobotContainer.m_OpticalLimitSwitch.isClosed());
    return RobotContainer.m_OpticalLimitSwitch.isClosed();
  }
}
