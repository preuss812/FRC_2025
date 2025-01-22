// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ColorDetectionSubsytem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class TakeInAlgaeCommand extends Command {
  private final AlgaeIntakeSubsystem m_AlgaeIntakeSubsystem;
  private final ColorDetectionSubsytem m_colorDetectionSubsytem;

  /** Creates a new TakeInAlgaeCommand. */
  public TakeInAlgaeCommand(
    AlgaeIntakeSubsystem AlgaeIntakeSubsystem,
    ColorDetectionSubsytem colorDetectionSubsytem
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_AlgaeIntakeSubsystem = AlgaeIntakeSubsystem;
    m_colorDetectionSubsytem = colorDetectionSubsytem;
    addRequirements(AlgaeIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_AlgaeIntakeSubsystem.pickUpAlgae();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_AlgaeIntakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: Figure out how to know when the robot has control of an algae ball.
    return m_colorDetectionSubsytem.inRange();
  }
}
