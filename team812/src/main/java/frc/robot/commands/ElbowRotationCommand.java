// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElbowRotationSubsystem;
import frc.robot.Constants.ElbowConstants;

public class ElbowRotationCommand extends Command {
  /** Creates a new ArmCommand. */
  private final ElbowRotationSubsystem m_elbowSubsystem;
  private final double m_targetPosition;
  private final boolean debug = true; // TODO: set to false once this command is debugged.

  /**
   * ElbowRotationCommand - rotate the elbow joint to the specified position.
   * @param subsystem - the elbow rotation subsystem
   * @param position - the position in degrees you want to elbow join to rotate to.
   */
  public ElbowRotationCommand(ElbowRotationSubsystem subsystem, double position) {
    m_elbowSubsystem = subsystem;
    // Memorize the target position making sure it is within the range the joint can move.
    m_targetPosition =  MathUtil.clamp(position, ElbowConstants.kElbowMinPosition, ElbowConstants.kElbowMaxPosition);
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elbowSubsystem.setTargetPosition(m_targetPosition);   // TODO: Does this need to be here? - dph 2023-03-01
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {} // No special action required.  The subsystem will control the elbow

  public boolean onTarget() {
    double error = m_elbowSubsystem.getPositionError();

    if (Math.abs(error) < ElbowConstants.kElbowThreshold) {
      return true;
    } else if (m_elbowSubsystem.getCurrentPosition() > m_targetPosition && m_elbowSubsystem.isRevLimitSwitchClosed()) {
      // We are hear because the elbow was rotating up (to lower encoder values) and we hit the limit switch.
      return true;
    } else if (m_elbowSubsystem.getCurrentPosition() < m_targetPosition && m_elbowSubsystem.isFwdLimitSwitchClosed()) {
      // We are here because the elbow was rotating up (to lower encoder values) and we hit the limit switch.
      return true;
    } else {
      return false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elbowSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget();
  }
}
