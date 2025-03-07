// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.ShoulderRotationSubsystem;

public class ShoulderRotationCommand extends Command {
  /** Creates a new ShooterCommand. */
  private double targetPosition;
  private final ShoulderRotationSubsystem m_ShoulderRotationSubsystem;

  /**
   * ShoulderRotationCommand
   * @param subsystem - the shoulder rotation subsystem
   * @param position - the angle in degrees to move the shoulder joint to.
   */
  public ShoulderRotationCommand(ShoulderRotationSubsystem subsystem, double position) {
    m_ShoulderRotationSubsystem = subsystem;
    targetPosition = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem); 
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShoulderRotationSubsystem.setTargetPosition(targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShoulderRotationSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_ShoulderRotationSubsystem.getPositionError()) <= ShoulderConstants.kShoulderRotationThreshold;
  }
}
