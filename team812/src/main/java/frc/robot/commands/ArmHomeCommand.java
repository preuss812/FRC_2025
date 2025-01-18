// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
//import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ElbowRotationSubsystem;

public class ArmHomeCommand extends Command {
  /** Creates a new ArmHomeCommand. */
  private final ElbowRotationSubsystem m_armSubsystem;

  public ArmHomeCommand(ElbowRotationSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("homearm", "starting");
    // Clear the 'homed' flag in the arm subsystem.  Without clearing the flag
    // The arm command would not perform the full initialization.
    m_armSubsystem.unsetHome();

     // Set the arm encoder value to the expected starting position
     m_armSubsystem.setSensorReference();
     
     // Set the goal arm position to be the starting position plus the full range of possible values.
     // This ensures that even if the arm is at the opposite end of its range of motion,
     // this goal will rotate us to the fully rotated position.
     //m_armSubsystem.setHomePosition(ArmConstants.kArmMaxPosition+ArmConstants.kArmRange);

     // Plan B: rotate slowly until we reach the limit switch
     m_armSubsystem.runMotor(ArmConstants.kArmHomeSpeed);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("homearm", "end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_armSubsystem.isFwdLimitSwitchClosed()) {
      m_armSubsystem.setHome(); 
      m_armSubsystem.setPosition(m_armSubsystem.getPosition());
      return true;
    }
    return false;
  }
}
