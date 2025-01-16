// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeIntakeCommand extends Command {
  /** Creates a new ArmCommand. */
  private final AlgaeIntakeSubsystem m_AlgaeIntakeSubsystem;
  private final boolean debug = false;

  public AlgaeIntakeCommand(AlgaeIntakeSubsystem subsystem) {
    m_AlgaeIntakeSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (debug) SmartDashboard.putString("AlgaeIntakeCommand", "started");
    m_AlgaeIntakeSubsystem.pickUpAlgae();    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (debug) SmartDashboard.putString("AlgaeIntakeCommand", "end");
    m_AlgaeIntakeSubsystem.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // TODO is there a definition of finished?
  }
}
