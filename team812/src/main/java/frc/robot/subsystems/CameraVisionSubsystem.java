// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraVisionSubsystem extends SubsystemBase {
  /** Creates a new CameraVisionSubsystem. */
  public PhotonCamera camera = new PhotonCamera("pv-812");
  private final boolean debug = false;

  public CameraVisionSubsystem() {
    //camera.setDriverMode(true); // a guess
    //camera.setPipelineIndex(0);
  }

  public boolean hasTargets ()
  {
    var result = camera.getLatestResult();
    var idx = camera.getPipelineIndex();
    if (debug) SmartDashboard.putNumber("pipeline",idx);
  //camera.setDriverMode(false); // a test if this does what we expect

    if (result.hasTargets()) {
      if (debug) SmartDashboard.putString("Visual Target", "Yes");
    } else {
      if (debug) SmartDashboard.putString("Visual Target", "No");
    }
    return result.hasTargets();
  }

  public PhotonTrackedTarget getBestTarget() {
    var result = camera.getLatestResult();
     boolean hasTargets = result.hasTargets(); // true or false
  
    // if camera sees something, then
    if (hasTargets){
      PhotonTrackedTarget target = result.getBestTarget();
      return target;
    }
    return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (50 /s)
   var result = camera.getLatestResult();

   boolean hasTargets = result.hasTargets(); // true or false

   // if camera sees something, then // TODO is this part even needed?
      if (hasTargets){
        PhotonTrackedTarget target = result.getBestTarget(); // use best target
        // calculate distance to to april tag
        final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(19);
        final double TARGET_HEIGHT_METERS = Units.inchesToMeters(27); // target taped to table (center @ 2'3")
        final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(4);
        // final double GOAL_RANGE_METERS = Units.feetToMeters(0.5);
        double distance =
          PhotonUtils.calculateDistanceToTargetMeters(
                  CAMERA_HEIGHT_METERS,
                  TARGET_HEIGHT_METERS,
                  CAMERA_PITCH_RADIANS,
                  Units.degreesToRadians(target.getPitch()));
                  
        if (debug) SmartDashboard.putNumber("April Tag Target Distance (Predicted) (in)", Units.metersToInches(distance));
        if (debug) SmartDashboard.putNumber("detected April tag pitch (degree)", target.getPitch());
      }
  }


}
