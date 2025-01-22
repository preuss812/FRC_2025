// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.RobotContainer;

public class TakeInAlgaeOLSCommand extends Command {
  private final AlgaeIntakeSubsystem m_AlgaeIntakeSubsystem;

  /** Creates a new TakeInAlgaeCommand. */
  public TakeInAlgaeOLSCommand(AlgaeIntakeSubsystem AlgaeIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_AlgaeIntakeSubsystem = AlgaeIntakeSubsystem;
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
    //return m_colorDetectionSubsytem.isOrange(m_colorDetectionSubsytem.get_color());
    SmartDashboard.putBoolean("OpticalLimit", RobotContainer.m_OpticalLimitSwitch.isClosed());
    return RobotContainer.m_OpticalLimitSwitch.isClosed();
  }
}
