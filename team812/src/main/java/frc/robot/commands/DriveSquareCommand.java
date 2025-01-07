package frc.robot.commands;

import frc.robot.Constants.VisionConstants.AprilTag;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Main;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class DriveSquareCommand extends Command{
    private final DriveSubsystemSRX robotDrive;
    public DriveSquareCommand(DriveSubsystemSRX robotDrive) {
        this.robotDrive = robotDrive;
  }
    
  @Override
  public void initialize() {
    this.robotDrive.drive(0, 1, 0, isScheduled(), isFinished());
  }
}
