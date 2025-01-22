// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.ShoulderRotationSubsystem;

public class ShoulderHomeCommand extends Command {
  /** Creates a new ShoulderHomeCommand. */
  private final ShoulderRotationSubsystem m_ShoulderRotationSubsystem;

  public ShoulderHomeCommand(ShoulderRotationSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShoulderRotationSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Clear the 'homed' flag in the shoulder rotation subsystem.  Without clearing the flag
    // The shoulder command would not perform the full initialization.
    m_ShoulderRotationSubsystem.unsetHomed();

     // Plan B: rotate slowly until we reach the limit switch
     m_ShoulderRotationSubsystem.runMotor(ShoulderConstants.kShoulderHomeSpeed);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
        m_ShoulderRotationSubsystem.stop(); // Just in case we are still running.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_ShoulderRotationSubsystem.isAtHome()) {
      m_ShoulderRotationSubsystem.stop(); // Stop the motor
      m_ShoulderRotationSubsystem.setHomed(); // Remeber that we have performed the homing operation.
      m_ShoulderRotationSubsystem.setTargetPosition(ShoulderConstants.kShoulderHomePosition); // Set the encoder target value to the home position.
      return true;
    }
    return false;
  }
}
