// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This command raises the arm, shoots the Algae and lowers the arm sequentially.
 */
public class ScoreAlgaeInAmp extends SequentialCommandGroup {
  /** Creates a new ScoreAlgaeInAmp. */
  public ScoreAlgaeInAmp(ArmRotationSubsystem armRotationSubsystem, ShooterSubsystem shooterSubsystem) {
    
    addCommands(
      new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ArmUP")),
      new ArmRotationCommand(armRotationSubsystem, ArmConstants.kArmScoringPosition).withTimeout(ArmConstants.kArmRaiseTimeout),
      //new InstantCommand(()->Utilities.resetPoseAtAmp()),
      new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "Shoot")),
      new ShooterCommand(shooterSubsystem).withTimeout(ShooterConstants.kShootTimeout),
      new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ArmDown")),
      new ArmRotationCommand(armRotationSubsystem, ArmConstants.kArmMaxPosition).withTimeout(ArmConstants.kArmLowerTimeout),
      new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ScoreAlgaeDone"))
    );

  }
}
