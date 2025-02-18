package frc.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * This class is useful for autonomous or semi autonomous commands
 * It supplies x,y,theta pid controllers and supports viewing the dirving on the shuffleboard
 * in simulation mode for faster debugging.
 * 
 */
public class PreussAutoDrive {

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    private final DrivingConfig config;
    private final PoseEstimatorSubsystem poseEstimatorSubsystem;
    private final DriveSubsystemSRX robotDrive;
    private final boolean simulation;
    private Pose2d simulatedRobotPose;


    public PreussAutoDrive(
        DriveSubsystemSRX robotDrive,
        PoseEstimatorSubsystem poseEstimatorSubsystem,
        DrivingConfig config,
        boolean simulation
    ) {
        
        this.robotDrive = robotDrive;
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        this.config = config;
        this.simulation = simulation;

        xController = new PIDController(
          config.getLinearP(),
          config.getLinearI(),
          config.getLinearD()
        );
        xController.setIZone(config.getLinearIZone());
        yController = new PIDController(
          config.getLinearP(), 
          config.getLinearI(),
          config.getLinearD()
        );
        yController.setIZone(config.getLinearIZone());

        rotationController = new PIDController(
          config.getAngularP(),
          config.getAngularI(),
          config.getAngularD()
        );
        yController.setIZone(config.getLinearIZone());

        rotationController.setTolerance(config.getAngularTolerance()); // did not work, dont understand yet
        rotationController.enableContinuousInput(-Math.PI, Math.PI); // Tell PID Controller to expect inputs between -180 and 180 degrees (in Radians).
    }

    public double calculateX(double input) {
        return  xController.calculate(input, 0);
    }
    
    public double calculateClampedX(double input) {
        return  MathUtil.clamp(xController.calculate(input, 0), -config.getMaxThrottle(), config.getMaxThrottle());
    }

    public double calculateY(double input) {
        return  yController.calculate(input, 0);
    }
    
    public double calculateClampedY(double input) {
        return  MathUtil.clamp(yController.calculate(input, 0), -config.getMaxThrottle(), config.getMaxThrottle());
    }
    
    public double calculateRotation(double input) {
        return  rotationController.calculate(input, 0);
    }

    public double calculateClampedRotation(double input) {
        return  MathUtil.clamp(rotationController.calculate(input, 0), -config.getMaxRotation(), config.getMaxRotation());
    }

    public void reset() {
        xController.reset();
        yController.reset();
        rotationController.reset();
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative, boolean rateLimit) {
        robotDrive.drive(xSpeed, ySpeed, rotationSpeed, fieldRelative, rateLimit);
        if (simulation) {
            // For debug, update the forced robot location based on the x,y,theta applied.
            double scale=0.06; // Rate that the simulation applies the changes to the robot's position.
            simulatedRobotPose = new Pose2d(simulatedRobotPose.getX()+xSpeed*scale
            , simulatedRobotPose.getY()+ySpeed*scale
            , simulatedRobotPose.getRotation().plus(new Rotation2d(rotationSpeed*scale)));

            poseEstimatorSubsystem.setCurrentPose(simulatedRobotPose);
        }
    }

    public void setCurrentPose(Pose2d pose) {
        this.simulatedRobotPose = pose;
        poseEstimatorSubsystem.setCurrentPose(simulatedRobotPose);
    } 
    public Pose2d getCurrentPose() {
        if (simulation) {
            if (true || simulatedRobotPose == null) {
                simulatedRobotPose = poseEstimatorSubsystem.getCurrentPose();
            }
            return simulatedRobotPose;
        } else {
            return poseEstimatorSubsystem.getCurrentPose();
        }
    }
}
