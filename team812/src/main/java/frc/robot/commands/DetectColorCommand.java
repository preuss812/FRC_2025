// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ColorDetectionSubsytem;

/**
 * This command curently never ends so the purpose is not clear.
 */
public class DetectColorCommand extends Command {
  /** Creates a new DetectColorCommand. */
  ColorDetectionSubsytem m_ColorDetector;
  public DetectColorCommand(ColorDetectionSubsytem subsystem) {
    m_ColorDetector = subsystem;
 //   addRequirements(subsystem); // March 13, 2024 - colordetector is no longer a subsystem, but a class
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ColorDetector.get_color();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
