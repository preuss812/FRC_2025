// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.RobotContainer;

public class TakeInAlgaeOLSCommand extends Command {
  private final AlgaeIntakeSubsystem m_AlgaeIntakeSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;

  /** Creates a new TakeInAlgaeCommand. */
  public TakeInAlgaeOLSCommand(
    AlgaeIntakeSubsystem AlgaeIntakeSubsystem,
    ShooterSubsystem shooterSubsystem
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_AlgaeIntakeSubsystem = AlgaeIntakeSubsystem;
    m_ShooterSubsystem = shooterSubsystem;
    addRequirements(AlgaeIntakeSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_AlgaeIntakeSubsystem.pickUpAlgae();
    m_ShooterSubsystem.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_AlgaeIntakeSubsystem.stop();
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
