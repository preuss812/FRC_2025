/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.SplineHelper;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.Constants.UltrasonicConstants;
import frc.robot.Constants.VisionConstants;
//import frc.robot.Constants.VisionConstants.AprilTag;
//import frc.robot.subsystems.AnalogUltrasonicDistanceSubsystem;
import frc.robot.subsystems.ElbowRotationSubsystem;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ShoulderRotationSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX.DrivingMode;
import frc.robot.subsystems.PingResponseUltrasonicSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.CompoundArmMovementCommand;
import frc.robot.commands.GotoAprilTagCommand;
import frc.robot.commands.GotoPoseCommand;
//import frc.robot.subsystems.CameraVisionSubsystem;
//import frc.robot.subsystems.ColorDetectionSubsytem;
import frc.robot.commands.ShoulderHomeCommand;
import frc.robot.commands.ElbowRotationCommand;
//import frc.robot.commands.DetectColorCommand;
import frc.robot.commands.DriveOnAprilTagProjectionCommand;
import frc.robot.commands.DriveRobotCommand;
import frc.robot.commands.ElbowHomeCommand;
import frc.robot.commands.ExpelAlgaeCommand;
//import frc.robot.commands.BadGotoAprilTagCommand;
import frc.robot.commands.OpticalLimitSwitch;
//import frc.robot.commands.FindAprilTagCommand;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkLowLevel.MotorType;
//import frc.robot.commands.GotoProcessorCommand;
//import frc.robot.commands.GotoPoseCommand;
//import frc.robot.commands.AlgaeIntakeCommand;
//import frc.robot.commands.RotateRobotAutoCommand;
import frc.robot.commands.RotateRobotCommand;
import frc.robot.commands.SetCurrentPoseCommand;
//import frc.robot.commands.ScoreAlgaeInProcessor;
//import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ResetDriveTrainCommand;
import frc.robot.commands.StopAllMotorsCommand;
import frc.robot.commands.ShoulderRotationCommand;
//import frc.robot.commands.StopRobotMotion;
//import frc.robot.commands.SwerveToProcessorCommand;
//import frc.robot.commands.SwerveToPoseCommand;
//import frc.robot.commands.SwerveToSourceCommand;
import frc.robot.commands.SwerveToProcessorCommand;
import frc.robot.commands.SwerveToProcessorCommand;
import frc.robot.commands.TakeInAlgaeOLSCommand;
//import frc.robot.commands.GotoSourceCommand;
//import frc.robot.commands.DetectColorCommand;
//import frc.robot.TrajectoryPlans;
//import frc.robot.commands.PushTowardsWall;
import frc.robot.commands.PushTowardsWallUltrasonic;
import frc.utils.TriggerButton;
import frc.utils.PoseEstimatorCamera;




/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);
  //private final DriveTrain m_DriveTrain = new DriveTrain();
  // The robot's subsystems
  public final static DriveSubsystemSRX m_robotDrive = new DriveSubsystemSRX();

  public static BlackBoxSubsystem m_BlackBox = new BlackBoxSubsystem();
  public static PoseEstimatorCamera m_rearCamera = new PoseEstimatorCamera("pv-812", VisionConstants.ROBOT_TO_REAR_CAMERA);
  //public static PoseEstimatorCamera m_frontCamera = new PoseEstimatorCamera("Microsoft_LifeCam_HD-3000", VisionConstants.ROBOT_TO_FRONT_CAMERA);

  //public static EncoderSubsystem m_EncoderSubsystem = new EncoderSubsystem();
public static final PoseEstimatorCamera[] cameras = new PoseEstimatorCamera[]{m_rearCamera/*m_frontCamera*/};
  public static PoseEstimatorSubsystem m_PoseEstimatorSubsystem = new PoseEstimatorSubsystem( cameras, m_robotDrive);
  public static ElbowRotationSubsystem m_ElbowRotationSubsystem = new ElbowRotationSubsystem();
  public static ShoulderRotationSubsystem m_ShoulderRotationSubsystem = new ShoulderRotationSubsystem();
  public static AlgaeIntakeSubsystem m_AlgaeIntakeSubsystem = new AlgaeIntakeSubsystem(Constants.algaeMotorConfig);
  private static  boolean isSimulation = false;
  
  public static final boolean isDebug = true;
  private static boolean debug = true && isDebug(); // To enable debugging in this module, change false to true.

  //public static PowerDistribution m_PowerDistribution = new PowerDistribution(0, ModuleType.kCTRE);
  //public static ColorDetectionSubsytem m_ColorDetectionSubsystem = new ColorDetectionSubsytem();
  //public static AnalogUltrasonicDistanceSubsystem m_UltrasonicDistanceSubsystem = new AnalogUltrasonicDistanceSubsystem();
  public static PingResponseUltrasonicSubsystem m_PingResponseUltrasonicSubsystem =
    new PingResponseUltrasonicSubsystem(
      UltrasonicConstants.kPingChannel,
      UltrasonicConstants.kEchoChannel,
      UltrasonicConstants.kOffsetToBumper
    );
  public static final OpticalLimitSwitch m_OpticalLimitSwitch = new OpticalLimitSwitch(AlgaeIntakeConstants.kLimitSwitchChannel);

  //public static DigitalIOSubsystem m_DigitalIOSubsystem = new DigitalIOSubsystem();

  // Controller definitions
  private final Joystick leftJoystick = new Joystick(OIConstants.kLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.kRightJoystick);
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  
  //public static CANcoder m_enctest = new CANcoder(38);
  //private final SendableChooser<Command> autoChooser;
  
  // private final Joystick xboxController = new
  // Joystick(OIConstants.kXboxController);
  double POV_to_double(int pov) {
    double result;
    if (pov == -1) {
      result = 0.0;
    } else if (pov == 0) {
      result = 0.5;
    } else if (pov == 180) {
      result = -0.5;
    } else {
      result = 0.0;
    }
    // SmartDashboard.putNumber("POV", pov);
    // SmartDashboard.putNumber("POV_Out", result);
    return result;
  }
  
  /**
   * Convenience function to create xbox-direction pad rotate buttons.
   * Comprehends alliance to turn to driver's field perspective.
   * @param heading
   * @return xbox-dPad button for turning to the specified heading.
   */
  POVButton dPadButton(int heading) {
    POVButton button = new POVButton(m_driverController, heading);
    button.onTrue(
      new RotateRobotCommand(
          m_robotDrive, 
          Units.degreesToRadians(-heading),
          false
        ).withTimeout(2.0)

    ).debounce(0.2);
    return button;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {    

        // By default this is not a simulation.
// For convenience, set the simulation mode to true if this is MacOS.
    RobotContainer.isSimulation = (System.getProperty("os.name").equals("Mac OS X"));
    TrajectoryPlans.buildAutoTrajectories(); 

    // Configure the button bindings
    configureButtonBindings();
    
    // Configure default commands

    // The xBox controller left stick controls translation of the robot.
    // The xBox controller right stick controls the direction the robot is facing (spinning).
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.allianceRelativeDrive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
            true,  true),
        m_robotDrive)
    );

    // The left joystick controls the rotation of the elbow.
    m_ElbowRotationSubsystem.setDefaultCommand(
      new RunCommand(() -> m_ElbowRotationSubsystem.rotate(-leftJoystick.getY()), m_ElbowRotationSubsystem)
    );
    
    
    // The right joystick controls the rotation of the shoulder.
    m_ShoulderRotationSubsystem.setDefaultCommand(
      new RunCommand(() -> m_ShoulderRotationSubsystem.rotate(-rightJoystick.getY()), m_ShoulderRotationSubsystem)
    );

    SmartDashboard.putData("SH+", new RunCommand(()->m_ShoulderRotationSubsystem.runMotor(1.0)));
    SmartDashboard.putData("SH-", new RunCommand(()->m_ShoulderRotationSubsystem.runMotor(-1.0)));
    SmartDashboard.putData("SH0",new InstantCommand(()->m_ShoulderRotationSubsystem.runMotor(0.0)));

    SmartDashboard.putData("EL+", new RunCommand(()->m_ElbowRotationSubsystem.runMotor(0.2), m_ElbowRotationSubsystem));
    SmartDashboard.putData("EL-", new RunCommand(()->m_ElbowRotationSubsystem.runMotor(-0.2), m_ElbowRotationSubsystem));
    SmartDashboard.putData("EL0",new InstantCommand(()->m_ElbowRotationSubsystem.runMotor(0.0), m_ElbowRotationSubsystem));
    
    SmartDashboard.putData("ELm45",new InstantCommand(()->m_ElbowRotationSubsystem.setTargetPosition(-45.0)));
    SmartDashboard.putData("ELp45",new InstantCommand(()->m_ElbowRotationSubsystem.setTargetPosition(45.0)));
    
    SmartDashboard.putData("ELm90",new InstantCommand(()->m_ElbowRotationSubsystem.setTargetPosition(-180.0)));
    SmartDashboard.putData("ELp90",new InstantCommand(()->m_ElbowRotationSubsystem.setTargetPosition(180.0)));
    
    /* Switched to TriggerButton
    // Default is to expel Algaes based on the percentage pulled of the left trigger.
    m_AlgaeIntakeSubsystem.setDefaultCommand(
      new RunCommand(()->m_AlgaeIntakeSubsystem.runMotor(-m_driverController.getLeftTriggerAxis()), m_AlgaeIntakeSubsystem)
    );
    */

    // Default is to score Algaes based on the percentage pulled of the left trigger.
    m_AlgaeIntakeSubsystem.setDefaultCommand(
      new RunCommand(()->m_AlgaeIntakeSubsystem.runMotor(m_driverController.getRightTriggerAxis()), m_AlgaeIntakeSubsystem)
    );

    Ultrasonic.setAutomaticMode(true);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    
    new JoystickButton(m_driverController, Button.kRightBumper.value)
      .whileTrue(new AlgaeIntakeCommand(m_AlgaeIntakeSubsystem));

    new TriggerButton(m_driverController, Axis.kRightTrigger)
      .whileTrue(new ExpelAlgaeCommand(m_AlgaeIntakeSubsystem));

    // Xbox A button spits out the algae
    new JoystickButton(m_driverController, Button.kA.value)
      .whileTrue(
        //new ExpelAlgaeCommand(m_AlgaeIntakeSubsystem)
        new GotoAprilTagCommand(m_robotDrive, m_PoseEstimatorSubsystem, Units.inchesToMeters(-2), null)
      );
    
      
      // Xbox Y button resets the robot coorinate system
    new JoystickButton(m_driverController, Button.kY.value).onTrue(new ResetDriveTrainCommand());

    // Xbox start button puts thte robot in fast/speed driving mode.
    new JoystickButton(m_driverController, Button.kStart.value).onTrue(
      new InstantCommand(()->m_robotDrive.setDrivingMode(DrivingMode.SPEED))
    );
    
        // Xbox start button puts thte robot in slow/precision driving mode.
    new JoystickButton(m_driverController, Button.kBack.value).onTrue(
      new InstantCommand(()->m_robotDrive.setDrivingMode(DrivingMode.PRECISION))
    );


    // POV buttons to point robot to a given heading where 0 is
    // straight downfield from the driver's perspective.
    // These change depending on blue or red alliance.
    POVButton   dPad0 = dPadButton(0);
    POVButton  dPad45 = dPadButton(45);
    POVButton  dPad90 = dPadButton(90);
    POVButton dPad135 = dPadButton(135);
    POVButton dPad180 = dPadButton(180);
    POVButton dPad225 = dPadButton(225);
    POVButton dPad270 = dPadButton(270);
    POVButton dPad315 = dPadButton(315);


    // Debugging commands for the elbow and shoulder.
    new JoystickButton(leftJoystick, 7).onTrue(new InstantCommand(() -> m_ElbowRotationSubsystem.testSetHomed()));
    new JoystickButton(rightJoystick, 7).onTrue(new InstantCommand(() -> m_ShoulderRotationSubsystem.testSetHomed()));
   
    new JoystickButton(leftJoystick, 8).onTrue(new InstantCommand(()->m_ElbowRotationSubsystem.setTargetPosition(ElbowConstants.kElbowHomePosition)));
    new JoystickButton(rightJoystick, 8).onTrue(new InstantCommand(()->m_ShoulderRotationSubsystem.setTargetPosition(ShoulderConstants.kShoulderHomePosition)));
    
    new JoystickButton(leftJoystick, 10).onTrue(new InstantCommand(()->m_ElbowRotationSubsystem.setTargetPosition(ElbowConstants.kElbowLowAlgaePosition)));
    new JoystickButton(rightJoystick, 10).onTrue(new InstantCommand(()->m_ShoulderRotationSubsystem.setTargetPosition(ShoulderConstants.kShoulderLowAlgaePosition)));
    
    new JoystickButton(leftJoystick, 12).onTrue(new InstantCommand(()->m_ElbowRotationSubsystem.setTargetPosition(ElbowConstants.kElbowHighAlgaePosition)));
    new JoystickButton(rightJoystick, 12).onTrue(new InstantCommand(()->m_ShoulderRotationSubsystem.setTargetPosition(ShoulderConstants.kShoulderHighAlgaePosition)));
    
    /* Debugging below */
    if (debug) {

      SmartDashboard.putData("IC", new InstantCommand(() -> m_PoseEstimatorSubsystem.setCurrentPose(new Pose2d(1,4,new Rotation2d(0)))));
      SmartDashboard.putData("EL", new ElbowRotationCommand(m_ElbowRotationSubsystem, 5000));

      /**
       * Create smart dash button that cycles through the various paths
       * from each 2x2 meter square on the field to the Processor and source
       * for both red and blue alliances.
       */
      
      if (debug) {
        boolean relative = true;
        SmartDashboard.putData("Dance", TrajectoryPlans.robotDanceCommand(m_robotDrive, m_PoseEstimatorSubsystem, null /*=default*/) );
        SmartDashboard.putData("S2P", new SwerveToProcessorCommand(m_robotDrive, m_PoseEstimatorSubsystem));
        SmartDashboard.putData("RRcmd", new RotateRobotCommand(m_robotDrive, Units.degreesToRadians(45.0), relative));
        SmartDashboard.putData("DRcmd", new DriveRobotCommand(
            m_robotDrive
          , m_PoseEstimatorSubsystem
          , new Pose2d(1.0,0.0, new Rotation2d()) // Forward 1 meter
          , false
          , null /*=default*/
        ));
        SmartDashboard.putData("G2A",new GotoAprilTagCommand(
            m_robotDrive
          , m_PoseEstimatorSubsystem
          , Units.inchesToMeters(5.0)
          , m_robotDrive.defaultAutoConfig
        ));
        SmartDashboard.putData("DOP",new DriveOnAprilTagProjectionCommand(
          m_PoseEstimatorSubsystem
          , m_robotDrive
          , m_driverController
          , m_robotDrive.defaultAutoConfig
          ));
        SmartDashboard.putData("SHP", new InstantCommand(() -> m_ShoulderRotationSubsystem.setSensorPosition(45)));
        SmartDashboard.putData("ELP", new InstantCommand(() -> m_ElbowRotationSubsystem.setSensorPosition(45)));
        SmartDashboard.putData("TEST", new DriveRobotCommand(
          m_robotDrive
          , m_PoseEstimatorSubsystem
          , new Pose2d(-4, 0, new Rotation2d(0)) // backward 4 meters.
          , false
          , null));
        SmartDashboard.putData("G2P", new GotoPoseCommand(
          m_robotDrive
          , m_PoseEstimatorSubsystem
          , new Pose2d(
            FieldConstants.blueStartLine
            , m_PoseEstimatorSubsystem.getAprilTagPose(14).getY()
            , new Rotation2d(0.0))
            , false 
            , null));
        SmartDashboard.putData("TTcmd", new SequentialCommandGroup(
            new SetCurrentPoseCommand(m_PoseEstimatorSubsystem)
          , new SwerveToProcessorCommand(m_robotDrive, m_PoseEstimatorSubsystem)
        ));
         SmartDashboard.putData("SCP", new SetCurrentPoseCommand(m_PoseEstimatorSubsystem));

        // Start the game
        SmartDashboard.putData("AP0",
          new CompoundArmMovementCommand(m_ElbowRotationSubsystem, m_ShoulderRotationSubsystem, Units.degreesToRadians(170), Units.degreesToRadians(60)));
        // Grab algae from lower level
        SmartDashboard.putData("AP1",
          new CompoundArmMovementCommand(m_ElbowRotationSubsystem, m_ShoulderRotationSubsystem, Units.degreesToRadians(180), Units.degreesToRadians(95)));
        // Pick up algae from ground
        SmartDashboard.putData("AP2",
          new CompoundArmMovementCommand(m_ElbowRotationSubsystem, m_ShoulderRotationSubsystem, Units.degreesToRadians(30), Units.degreesToRadians(10)));
        // Score algae in processor 
        SmartDashboard.putData("AP3",
          new CompoundArmMovementCommand(m_ElbowRotationSubsystem, m_ShoulderRotationSubsystem, Units.degreesToRadians(90), Units.degreesToRadians(5)));
        // Climbing
        SmartDashboard.putData("AP4",
          new CompoundArmMovementCommand(m_ElbowRotationSubsystem, m_ShoulderRotationSubsystem, Units.degreesToRadians(-45),Units.degreesToRadians(95.0)));

          /*SmartDashboard.putData("AA4", new ConditionalCommand(
            TrajectoryPlans.blueAutoPlans.get(4),
            TrajectoryPlans.redAutoPlans.get(4),
            () -> Utilities.isBlueAlliance()
          ));
          */
          //SmartDashboard.putData("AA5", TrajectoryPlans.autoPlans.get(5));
          //SmartDashboard.putData("AA6", TrajectoryPlans.autoPlans.get(6));
          //SmartDashboard.putData("AA7", TrajectoryPlans.autoPlans.get(7));
        }
    } // (debug)
  } // (configureButtonBindings)

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Autonomous(this);
  }
   
  // Function to align the PoseEstimator pose and the DriveTrain pose.
  // This assumes that the PoseEstimator has a really good estimate.
  // In other words, that it has a recent, accurate view of an Apriltag.
  public void alignDriveTrainToPoseEstimator() {
    // Set the gyro angle to match the pose estimator 
    // compensating for the placement of the camera on the robot.
    /*
    m_robotDrive.setAngleDegrees( 
      MathUtil.inputModulus(
        m_PoseEstimatorSubsystem.getCurrentPose().getRotation().getDegrees()+VisionConstants.CAMERA_TO_ROBOT.getRotation().toRotation2d().getDegrees()
        ,-180
        , 180
      )
    );
    */
    // Update the drive trains X, Y, and robot orientation to match the pose estimator.
    m_robotDrive.resetOdometry(m_PoseEstimatorSubsystem.getCurrentPose());
  }

  /**
   *  This function sets the gyro angle based on the alliance (blue or red)
   * and the assumed starting position of the robot on the field.
   * The current assumption is that the robot will be placed with it's back to the
   * alliance wall.  The "Y" coordinates of the robot will be determined by the
   * PoseEstimator once an april tag is captured by the vision system.
   */
   public static void setGyroAngleToStartMatch() {
    boolean isBlueAlliance = Utilities.isBlueAlliance(); // From the Field Management system.
    double startingHeading; // degrees.
    if (isBlueAlliance) {
      startingHeading = 0.0;
      m_robotDrive.setAngleDegrees(startingHeading);
      m_robotDrive.resetOdometry(
        new Pose2d(
           DriveConstants.kBackToCenterDistance // Assume robot is back to the starting wall
          ,FieldConstants.yCenter               // Pick center of the field since we dont know where we will start.
          ,new Rotation2d(Units.degreesToRadians(startingHeading)) // Facing toward the field.
        )
      );
    } else {
      startingHeading = 180.0;
      m_robotDrive.setAngleDegrees(startingHeading);
      m_robotDrive.resetOdometry(
        new Pose2d(
           FieldConstants.xMax - DriveConstants.kBackToCenterDistance // Assume robot is back to the starting wall
          ,FieldConstants.yCenter                                     // Pick center of the field since we dont know where we will start.
          ,new Rotation2d(Units.degreesToRadians(startingHeading))    // Facing toward the field.
        )
      );
    }
   }
   /**
   *  This function sets the gyro angle based on the alliance (blue or red)
   * and the assumed starting position of the robot on the field.
   * The current assumption is that the robot will be placed with it's back 
   * close to the Processor wall. The X coodinate will be calculate based on the
   * assumption that the robot is just inside the starting box.
   */
  public static void setGyroAngleToStartMatchProcessor() {
    double ProcessorZoneDepth = Units.inchesToMeters(13);
    double startingBoxDepth = Units.inchesToMeters(76.1);
    double startingRobotCenterY = FieldConstants.yMax - ProcessorZoneDepth - DriveConstants.kBackToCenterDistance;
    double startingRobotCenterX = FieldConstants.xMin + startingBoxDepth - DriveConstants.kRobotWidth/2.0 - Units.inchesToMeters(2.0)/* tape */;

    boolean isBlueAlliance = Utilities.isBlueAlliance(); // From the Field Management system.
    double startingHeading; // degrees.
    if (isBlueAlliance) {
      startingHeading = 90; // Facing source wall
      m_robotDrive.setAngleDegrees(startingHeading);
      m_robotDrive.resetOdometry(
        new Pose2d(
           startingRobotCenterX     // Just inside the starting box near the Processor
          ,startingRobotCenterY     // Just inside the starting box near the Processor
          ,new Rotation2d(Units.degreesToRadians(startingHeading)) // Facing toward the field.
        )
      );
    } else {
      startingHeading = 90.0; // Facing source wall
      m_robotDrive.setAngleDegrees(startingHeading);
      m_robotDrive.resetOdometry(
        new Pose2d(
           FieldConstants.xMax - startingRobotCenterX     // Just inside the starting box near the Processor
          ,startingRobotCenterY                           // Just inside the starting box near the Processor
          ,new Rotation2d(Units.degreesToRadians(startingHeading))    // Facing toward the field.
        )
      );
    }
   }

   /**
    * isSimulation - return true if this is a simulation or false if the robot is running.
    */
    public static boolean isSimulation() {
      return isSimulation;
    }

    /**
    * isDebug - return true if debugging output is enabled.
    */
    public static boolean isDebug() {
      return isDebug;
    }
}