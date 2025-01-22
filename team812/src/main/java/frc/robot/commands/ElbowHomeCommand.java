// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElbowConstants;
import frc.robot.subsystems.ElbowRotationSubsystem;

public class ElbowHomeCommand extends Command {
  /** Creates a new ElbowHomeCommand. */
  private final ElbowRotationSubsystem m_ElbowRotationSubsystem;

  public ElbowHomeCommand(ElbowRotationSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ElbowRotationSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Clear the 'homed' flag in the elbow rotation subsystem.  Without clearing the flag
    // The elbow command would not perform the full initialization.
    m_ElbowRotationSubsystem.unsetHomed();

     // Plan B: rotate slowly until we reach the limit switch
     m_ElbowRotationSubsystem.runMotor(ElbowConstants.kElbowHomeSpeed);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElbowRotationSubsystem.stop(); // Just in case we are still running.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_ElbowRotationSubsystem.isAtHome()) {
      m_ElbowRotationSubsystem.stop(); // Stop the motor
      m_ElbowRotationSubsystem.setHomed(); // Remeber that we have performed the homing operation.
      m_ElbowRotationSubsystem.setTargetPosition(ElbowConstants.kElbowHomePosition); // Set the encoder target value to the home position.
      return true;
    }
    return false;
  }
}
