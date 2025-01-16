// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class ExpelAlgaeCommand extends Command {
  /** Creates a new ExpelAlgaeCommand. */
  private final AlgaeIntakeSubsystem AlgaeIntakeSubsystem;
  public ExpelAlgaeCommand(AlgaeIntakeSubsystem AlgaeIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.AlgaeIntakeSubsystem = AlgaeIntakeSubsystem;
    addRequirements(AlgaeIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AlgaeIntakeSubsystem.expelAlgae(); // This sets the motor speed.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    AlgaeIntakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  // There is no finish for this one, use .withTimeout or .whileTrue.
  }
}
