// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.io.IOException;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private SparkMax leftDrive;
  private SparkMax rightDrive;
  private DifferentialDrive differentialDrive;
  private RobotConfig config = null;
  private DifferentialDriveKinematics kinematics;
  private Pose2d pose;
  private SparkMaxConfig leftConfig;
  private SparkMaxConfig rightConfig;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private SparkClosedLoopController leftController;
  private SparkClosedLoopController rightController;
  
  private double lastLeftPositionMeters = 0.0;
  private double lastRightPositionMeters = 0.0;

  /** Creates a new Drive. */
  public Drivetrain() {
      leftDrive = new SparkMax(Constants.MotorConstants.LEFT_MOTOR_ID,MotorType.kBrushless);
      leftEncoder = leftDrive.getEncoder();
      leftController = leftDrive.getClosedLoopController();

      leftConfig = new SparkMaxConfig();
      leftConfig.inverted(Constants.MotorConstants.LEFT_MOTOR_INVERTED);
      leftConfig.smartCurrentLimit(Constants.MotorConstants.LEFT_MOTOR_AMP_LIMIT);
      leftConfig.closedLoop
        .p(Constants.MotorConstants.DRIVE_P)
        .i(Constants.MotorConstants.DRIVE_I)
        .d(Constants.MotorConstants.DRIVE_D)
        .velocityFF(Constants.MotorConstants.DRIVE_FF)
        .outputRange(0.0, 1.0);
      leftDrive.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


      rightDrive = new SparkMax(Constants.MotorConstants.RIGHT_MOTOR_ID,MotorType.kBrushless);
      rightController = rightDrive.getClosedLoopController();
      rightEncoder = rightDrive.getEncoder();

      rightConfig = new SparkMaxConfig();
      rightConfig.inverted(Constants.MotorConstants.RIGHT_MOTOR_INVERTED);
      rightConfig.smartCurrentLimit(Constants.MotorConstants.RIGHT_MOTOR_AMP_LIMIT);

      rightDrive.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


      differentialDrive = new DifferentialDrive(leftDrive, rightDrive);
      addChild("Differential Drive 1", differentialDrive);
      differentialDrive.setExpiration(0.1);
      differentialDrive.setMaxOutput(1.0);

      kinematics = new DifferentialDriveKinematics(0.8204);
      pose = new Pose2d();
      
      try {
        config = RobotConfig.fromGUISettings();
      } catch (IOException e) {
        e.printStackTrace();
      } catch (ParseException e) {
        e.printStackTrace();
      }
    

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double leftPositionMetersDelta =
      getLeftPositionMeters() - lastLeftPositionMeters;
    double rightPositionMetersDelta =
      getRightPositionMeters() - lastRightPositionMeters;
    double avgPositionMetersDelta =
      (leftPositionMetersDelta + rightPositionMetersDelta) / 2.0;

    // Update the pose based on the average position delta
    pose = pose.exp(new Twist2d(
      avgPositionMetersDelta,
      0.0,
      (rightPositionMetersDelta - leftPositionMetersDelta)
                  / Constants.DriveConstants.TRACK_WIDTH_METERS));
    
    lastLeftPositionMeters = getLeftPositionMeters();
    lastRightPositionMeters = getRightPositionMeters();
  }

  public double getLeftPositionMeters() {
    return leftEncoder.getPosition() * Constants.DriveConstants.WHEEL_DIAMETER_METERS;
  }
  public double getRightPositionMeters() {
    return rightEncoder.getPosition() * Constants.DriveConstants.WHEEL_DIAMETER_METERS;
  }

  // public void setDrive(double left, double right) {
  //   if (invert == false) {
  //       differentialDrive.tankDrive(-left, right, true);
  //       SmartDashboard.putNumber("Left Drive", -left);
  //       SmartDashboard.putNumber("Right Drive", right);
  //   } else {
  //       differentialDrive.tankDrive(right, -left, true);
  //       SmartDashboard.putNumber("Left Drive", -left);
  //       SmartDashboard.putNumber("Right Drive", right);
  //   }  
  // }
  public void setMotors(double left, double right) {
    setLeftMotors(left);
    setRightMotors(right);
  }
  public void setLeftMotors (double volt) {
    leftDrive.set(volt);
    
  }
  public void setRightMotors (double volt) {
    rightDrive.set(volt);
  }

  public Pose2d getPose()
  {
    return pose;
  }
  public Pose2d resetPose(Pose2d newPose)
  {
    pose = newPose;
    return pose;
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeed(){
    return new DifferentialDriveWheelSpeeds(leftDrive.getEncoder().getVelocity(), rightDrive.getEncoder().getVelocity());
  }
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return kinematics.toChassisSpeeds(getWheelSpeed());
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    DifferentialDriveWheelSpeeds tankWheelsSpeeds = kinematics.toWheelSpeeds(speeds);
    tankWheelsSpeeds.desaturate(0.75);

    differentialDrive.tankDrive(-tankWheelsSpeeds.leftMetersPerSecond, -tankWheelsSpeeds.rightMetersPerSecond);
  }

  public void drive(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    DifferentialDriveWheelSpeeds tankWheelsSpeeds = kinematics.toWheelSpeeds(speeds);
    tankWheelsSpeeds.desaturate(0.75);

    differentialDrive.tankDrive(-tankWheelsSpeeds.leftMetersPerSecond, -tankWheelsSpeeds.rightMetersPerSecond);
  }

  public Command followPathCommand(String pathName) {
    try{
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return new FollowPathCommand(
                path,
                this::getPose, // Robot pose supplier
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
                config, // The robot configuration
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
        );
    } catch (Exception e){
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }
}