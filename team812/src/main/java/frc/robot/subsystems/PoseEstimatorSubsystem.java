// Code from frc-7028-2023/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java
// Copied Jan 22, 2024 as a seed for our Pose Estimator.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;

import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;
import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.DriveTrainConstants;
//import frc.robot.subsystems.DriveSubsystemSRX;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final PhotonCamera photonCamera;
  private final DriveSubsystemSRX drivetrainSubsystem;
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private boolean debug = false;
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  public final Field2d field2d = new Field2d();
  public final Trajectory trajectory = null;

  private double previousPipelineTimestamp = 0;
  private boolean isBlueAlliance = true;
  private int m_lastAprilTagSeen = 0;

  public PoseEstimatorSubsystem(PhotonCamera photonCamera, DriveSubsystemSRX drivetrainSubsystem) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    AprilTagFieldLayout layout;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      //var alliance = DriverStation.getAlliance();
      //isBlueAlliance = (alliance.get() == Alliance.Blue);  // Remember which alliance we are in.
      // TODO: Figure out if this is needed and fix if necessary
      //layout.setOrigin(alliance == Alliance.Blue ?
      //    OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    this.aprilTagFieldLayout = layout;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator =  new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        drivetrainSubsystem.getRotation(),
        drivetrainSubsystem.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
    
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
    if (trajectory != null) 
      field2d.getObject("trajectory").setTrajectory(trajectory);

  }

  @Override
  public void periodic() {
    Utilities.toSmartDashboard("PE CurrentPose", getCurrentPose());

    // Update pose estimator with the best visible target
    var pipelineResult = photonCamera.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();
    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
      if (debug) SmartDashboard.putNumber("TagAmbiguity", target.getPoseAmbiguity());
      if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
        var targetPose = tagPose.get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
        if (debug)
          Utilities.toSmartDashboard("PE pose",visionMeasurement.toPose2d());
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
        m_lastAprilTagSeen = fiducialId;
      } else {
        m_lastAprilTagSeen = -1;
      }
    } else if (resultTimestamp == previousPipelineTimestamp && pipelineResult.hasTargets()) {
        // Do nothing, we have the same result as the last call so whatever the result was then is still the result now.
    } else {
      m_lastAprilTagSeen = 0;
    }
    
    if (debug) SmartDashboard.putBoolean("TagDetected", m_lastAprilTagSeen != 0);
    if (debug) SmartDashboard.putNumber("TagID", m_lastAprilTagSeen);

    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
      drivetrainSubsystem.getRotation(),
      drivetrainSubsystem.getModulePositions());

    field2d.setRobotPose(getCurrentPose());
    if (trajectory != null) 
      field2d.getObject("trajectory").setTrajectory(trajectory);
      
  }

  public String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      drivetrainSubsystem.getRotation(),
      drivetrainSubsystem.getModulePositions(),
      newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public boolean isBlueAlliance () {
    return this.isBlueAlliance;
  }

  public Pose2d getAprilTagPose(int aprilTagId) {
    Optional<Pose3d> pose3d;
    Pose2d pose2d = new Pose2d();
    pose3d = aprilTagFieldLayout.getTagPose(aprilTagId);
    if (pose3d.isPresent())
      pose2d = pose3d.get().toPose2d();
    return pose2d;
  }

  public int lastAprilTagSeen() {
    return m_lastAprilTagSeen;
  }

  public boolean tagInView() {
    return (m_lastAprilTagSeen > 0);
  }

  public BooleanSupplier tagInViewSupplier = () -> (m_lastAprilTagSeen > 0);

}
