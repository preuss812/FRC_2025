// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElbowRotationSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmRotationCommand extends Command {
  /** Creates a new ArmCommand. */
  private final ElbowRotationSubsystem m_armSubsystem;
  private final double m_position;
  private double setPoint;
  private final boolean debug = false;

  public ArmRotationCommand(ElbowRotationSubsystem subsystem, double position) {
    m_armSubsystem = subsystem;
    m_position = position;
    System.out.println("ArmCommand class setPoint is " + m_position);
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (debug) SmartDashboard.putString("armcmd", "started");

    setPoint = MathUtil.clamp(m_position, ArmConstants.kArmMinPosition, ArmConstants.kArmMaxPosition);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setPosition(setPoint);   // TODO: Does this need to be here? - dph 2023-03-01
  }

  public boolean onTarget() {
    double error = m_armSubsystem.getPosition() - setPoint;
    if (debug) SmartDashboard.putNumber("armcmderr", error);

    if (Math.abs(error) < ArmConstants.kArmThreshold) {
      return true;
    } else if (m_armSubsystem.getPosition() > setPoint && m_armSubsystem.isRevLimitSwitchClosed()) {
      // We are hear because the arm was rotating up (to lower encoder values) and we hit the limit switch.
      return true;
    } else if (m_armSubsystem.getPosition() < setPoint && m_armSubsystem.isFwdLimitSwitchClosed()) {
      // We are here because the arm was rotating up (to lower encoder values) and we hit the limit switch.
      return true;
    } else {
      return false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (debug) SmartDashboard.putString("armcmd", "end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget();
  }
}
