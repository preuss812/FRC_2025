// Code from frc-7028-2023/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java
// Copied Jan 22, 2024 as a seed for our Pose Estimator.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
//import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final PhotonCamera photonCamera;
  private final DriveSubsystemSRX drivetrainSubsystem;
  
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private boolean debug =  false && RobotContainer.isDebug();
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
  private static StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("/MyPose", Pose2d.struct).publish();

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  public final Field2d field2d = new Field2d();
  //public final Trajectory trajectory = null;

  private double previousPipelineTimestamp = 0;
  private boolean isBlueAlliance = true;
  
  private int m_lastAprilTagSeen = VisionConstants.NO_TAG_FOUND;
  private static final PhotonPipelineResult m_emptyPipeline = new PhotonPipelineResult();
  private PhotonPipelineResult lastPipeLineResult = m_emptyPipeline;


  public PoseEstimatorSubsystem(PhotonCamera photonCamera, DriveSubsystemSRX drivetrainSubsystem) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    AprilTagFieldLayout layout;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
      // We are always using the blue alliance as the origin of the field.
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    this.aprilTagFieldLayout = layout;

    poseEstimator =  new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        drivetrainSubsystem.getRotation(),
        drivetrainSubsystem.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
    
    //tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    //tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
    SmartDashboard.putData("Field", field2d);
    //if (trajectory != null) 
    //  field2d.getObject("trajectory").setTrajectory(trajectory);

  }

  @Override
  public void periodic() {
    Utilities.toSmartDashboard("PE CurrentPose", getCurrentPose());

    // Update pose estimator with the best visible target
    var pipelineResult = getLatestResult();

    var resultTimestamp = pipelineResult.getTimestampSeconds();
    if (resultTimestamp > previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimestamp;

      var targets = pipelineResult.getTargets();
      
      //var target = pipelineResult.getBestTarget();
      var target = getBestTarget(targets);
      if (target == null) return;  // No targets => nothing to do.

      var fiducialId = target.getFiducialId();
      // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
      if (debug) SmartDashboard.putNumber("TagAmbiguity", target.getPoseAmbiguity());

      // If we have a field location for this tag, use it to update the robot's position.
      if (tagPose.isPresent()) {
        var targetPose = tagPose.get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
        if (debug)
          Utilities.toSmartDashboard("PE pose",visionMeasurement.toPose2d());
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
      }
    }
    
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
      drivetrainSubsystem.getRotation(),
      drivetrainSubsystem.getModulePositions());

    // Update the Shuffleboard with the robot's position on the field
    field2d.setRobotPose(getCurrentPose());
    publisher.set(getCurrentPose());

    //if (trajectory != null) 
    //  field2d.getObject("trajectory").setTrajectory(trajectory);
      
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

  // For autonomous to prevent motion if the robot cannot see an apriltag and therefore is in an unknown location.
  public BooleanSupplier tagInViewSupplier = () -> (m_lastAprilTagSeen > 0);

  // Return the pose for a robot to be directly in front of the specified apriltag
  public Pose2d robotFrontAtApriltag(int id, double offset) {
    return DriveConstants.robotFrontAtPose(getAprilTagPose(id), offset);
  }

  // Return the pose for a robot to be directly in front of the specified apriltag
  public Pose2d robotRearAtApriltag(int id, double offset) {
    return DriveConstants.robotRearAtPose(getAprilTagPose(id), offset);
  }  
  
  /**
   * getLatestResult - returns the most recent result from photonVision
   * @return - PhotonPipeLineResult() which may or may not have detected april tags depending on what the robot sees.
   */
  public PhotonPipelineResult getLatestResult() {
        PhotonPipelineResult result = lastPipeLineResult;

        // Grab the latest result. We don't care about the timestamp from NT - the metadata header has
        // this, latency compensated by the Time Sync Client
        var results = photonCamera.getAllUnreadResults();
        if (debug) SmartDashboard.putNumber("PV size", results.size());
        // This is different than in 2024.
        for (PhotonPipelineResult pipelineResult : results) {
          if (pipelineResult.hasTargets()) {
            //var target = pipelineResult.getBestTarget();
            var target = getBestTarget((pipelineResult.getTargets()));
            var bestFiducialId = target.getFiducialId();
            SmartDashboard.putNumber("PV ambiguity", target.getPoseAmbiguity());

            if (target.getPoseAmbiguity() <= VisionConstants.maximumAmbiguity 
            && bestFiducialId >= VisionConstants.MIN_FIDUCIAL_ID 
            && bestFiducialId <= VisionConstants.MAX_FIDUCIAL_ID) {
              result = pipelineResult;
              m_lastAprilTagSeen = bestFiducialId;
              lastPipeLineResult = pipelineResult;
            } else {
              m_lastAprilTagSeen = VisionConstants.NO_TAG_FOUND;
            }
          } else {
            m_lastAprilTagSeen = VisionConstants.NO_TAG_FOUND;
          }
        }
        if (debug) SmartDashboard.putNumber("PV last", m_lastAprilTagSeen);
        return result;
    }

    /**
     * find the best april tag for our purposes.
     * For 2025 that is the april tag with acceptable ambiguity 
     * having the closest to the center of the field of view
     * else largest area.
     * @param targets - the list of targets seen.  Typically there will be only 1.
     * @return - the best target.
     */
    public PhotonTrackedTarget getBestTarget(List<PhotonTrackedTarget> targets) {
      PhotonTrackedTarget result = null;
      double bestYaw = Math.PI; // Sentinel value. cannt have a greater heading than +/- field of view which is 100 degrees.
      double bestArea = 0.0;    // Smallest possible area.

      String s = "";

      //double bestAmbiguity = 1; // Sentinel value. cannot be more than 100% ambiguous.
      int targetsSeen = 0;
      for (PhotonTrackedTarget t : targets) {
        if (t.getPoseAmbiguity() <= VisionConstants.maximumAmbiguity
        && t.getFiducialId() >= VisionConstants.MIN_FIDUCIAL_ID 
        && t.getFiducialId() <= VisionConstants.MAX_FIDUCIAL_ID ) {
          targetsSeen++;
          if (targetsSeen == 1) {
            // First seen, therefore it must be the best so far :-)
            bestYaw = Math.abs(t.getYaw());
            bestArea = t.getArea();
            //bestAmbiguity = t.getPoseAmbiguity();
            result = t;
            s += t.getFiducialId()+":"+t.getYaw()+"   ";
          /* } else if (t.getPoseAmbiguity() < bestAmbiguity) {
            // Lower ambiguity so better for calculating location
            bestYaw = t.getYaw();
            bestArea = t.getArea();
            bestAmbiguity = t.getPoseAmbiguity();
            result = t; 
            targetsSeen++;*/
          } else if (Math.abs(t.getYaw()) < bestYaw) {
            // closest to the center of the field of view 
            bestYaw = Math.abs(t.getYaw());
            bestArea = t.getArea();
            //bestAmbiguity = t.getPoseAmbiguity();
            result = t;
            s += t.getFiducialId()+":"+t.getYaw()+"   ";

          } else if (Math.abs(t.getYaw()) == bestYaw && t.getArea() > bestArea) {
            // biggest => closest
            bestYaw = Math.abs(t.getYaw());
            bestArea = t.getArea();
            //bestAmbiguity = t.getPoseAmbiguity();
            result = t;
            s += t.getFiducialId()+":"+t.getYaw()+"   ";

          }
        }
      }
      SmartDashboard.putString("PV s", s);
      return result;
    }
}
