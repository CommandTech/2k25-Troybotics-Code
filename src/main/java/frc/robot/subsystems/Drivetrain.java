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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

public class Drivetrain extends SubsystemBase {
  private SparkMax leftDriveL;
  private SparkMax leftDriveF;
  private SparkMax rightDriveL;
  private SparkMax rightDriveF;
  private DifferentialDrive differentialDrive;
  private RobotConfig config;
  private DifferentialDriveKinematics kinematics;
  private Pose2d pose;
  private SparkMaxConfig leftConfigL;
  private SparkMaxConfig leftConfigF;
  private SparkMaxConfig rightConfigL;
  private SparkMaxConfig rightConfigF;
  private RelativeEncoder leftEncoderL;
  private RelativeEncoder leftEncoderF;
  private RelativeEncoder rightEncoderL;
  private RelativeEncoder rightEncoderF;
  private SparkClosedLoopController leftControllerL;
  private SparkClosedLoopController leftControllerF;
  private SparkClosedLoopController rightControllerL;
  private SparkClosedLoopController rightControllerF;
  private DifferentialDrivePoseEstimator poseEstimator;
  private DifferentialDriveWheelPositions wheelPositions;
  
  private double lastLeftPositionMeters = 0.0;
  private double lastRightPositionMeters = 0.0;

  /** Creates a new Drive. */
  public Drivetrain() {
      leftDriveL = new SparkMax(Constants.MotorConstants.LEADER_LEFT_MOTOR_ID,MotorType.kBrushless);
      leftEncoderL = leftDriveL.getEncoder();
      leftControllerL = leftDriveL.getClosedLoopController();

      leftConfigL = new SparkMaxConfig();
      leftConfigL.inverted(Constants.MotorConstants.LEADER_LEFT_MOTOR_INVERTED);
      leftConfigL.smartCurrentLimit(Constants.MotorConstants.LEADER_LEFT_MOTOR_AMP_LIMIT);
      leftConfigL.closedLoop
        .p(Constants.MotorConstants.DRIVE_P)
        .i(Constants.MotorConstants.DRIVE_I)
        .d(Constants.MotorConstants.DRIVE_D)
        .velocityFF(Constants.MotorConstants.DRIVE_FF)
        .outputRange(0.0, 1.0);
      leftDriveL.configure(leftConfigL, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

      leftDriveF = new SparkMax(Constants.MotorConstants.FOLLOWER_LEFT_MOTOR_ID,MotorType.kBrushless);
      leftEncoderF = leftDriveF.getEncoder();
      leftControllerF = leftDriveF.getClosedLoopController();

      leftConfigF = new SparkMaxConfig();
      leftConfigF.inverted(Constants.MotorConstants.FOLLOWER_LEFT_MOTOR_INVERTED);
      leftConfigF.smartCurrentLimit(Constants.MotorConstants.FOLLOWER_LEFT_MOTOR_AMP_LIMIT);
      leftConfigF.closedLoop
        .p(Constants.MotorConstants.DRIVE_P)
        .i(Constants.MotorConstants.DRIVE_I)
        .d(Constants.MotorConstants.DRIVE_D)
        .velocityFF(Constants.MotorConstants.DRIVE_FF)
        .outputRange(0.0, 1.0);
      leftDriveF.configure(leftConfigF, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      

      rightDriveL = new SparkMax(Constants.MotorConstants.LEADER_RIGHT_MOTOR_ID,MotorType.kBrushless);
      rightEncoderL = rightDriveL.getEncoder();
      rightControllerL = rightDriveL.getClosedLoopController();

      rightConfigL = new SparkMaxConfig();
      rightConfigL.inverted(Constants.MotorConstants.LEADER_RIGHT_MOTOR_INVERTED);
      rightConfigL.smartCurrentLimit(Constants.MotorConstants.LEADER_RIGHT_MOTOR_AMP_LIMIT);
      rightConfigL.closedLoop
        .p(Constants.MotorConstants.DRIVE_P)
        .i(Constants.MotorConstants.DRIVE_I)
        .d(Constants.MotorConstants.DRIVE_D)
        .velocityFF(Constants.MotorConstants.DRIVE_FF)
        .outputRange(0.0, 1.0);
      rightDriveL.configure(rightConfigL, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

      rightDriveF = new SparkMax(Constants.MotorConstants.FOLLOWER_RIGHT_MOTOR_ID,MotorType.kBrushless);
      rightEncoderF = rightDriveF.getEncoder();
      rightControllerF = rightDriveF.getClosedLoopController();

      rightConfigF = new SparkMaxConfig();
      rightConfigF.inverted(Constants.MotorConstants.FOLLOWER_RIGHT_MOTOR_INVERTED);
      rightConfigF.smartCurrentLimit(Constants.MotorConstants.FOLLOWER_RIGHT_MOTOR_AMP_LIMIT);
      rightConfigF.closedLoop
        .p(Constants.MotorConstants.DRIVE_P)
        .i(Constants.MotorConstants.DRIVE_I)
        .d(Constants.MotorConstants.DRIVE_D)
        .velocityFF(Constants.MotorConstants.DRIVE_FF)
        .outputRange(0.0, 1.0);
      rightDriveF.configure(rightConfigF, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      

      differentialDrive = new DifferentialDrive(leftDriveL, rightDriveL);
      addChild("Differential Drive 1", differentialDrive);
      differentialDrive.setExpiration(0.1);
      differentialDrive.setMaxOutput(1.0);

      kinematics = new DifferentialDriveKinematics(0.8204);
      pose = new Pose2d();
      wheelPositions = new DifferentialDriveWheelPositions(leftEncoderL.getPosition(), rightEncoderL.getPosition());

      poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics,
        new Rotation2d(),
        leftEncoderL.getPosition(),
        rightEncoderL.getPosition(),
        pose
      );

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
    // // This method will be called once per scheduler run
    // double leftPositionMetersDelta =
    //   getLeftPositionMeters() - lastLeftPositionMeters;
    // double rightPositionMetersDelta =
    //   getRightPositionMeters() - lastRightPositionMeters;
    // double avgPositionMetersDelta =
    //   (leftPositionMetersDelta + rightPositionMetersDelta) / 2.0;

    // // Update the pose based on the average position delta
    // pose = pose.exp(new Twist2d(
    //   avgPositionMetersDelta,
    //   0.0,
    //   (rightPositionMetersDelta - leftPositionMetersDelta)
    //               / Constants.DriveConstants.TRACK_WIDTH_METERS));
    
    // lastLeftPositionMeters = getLeftPositionMeters();
    // lastRightPositionMeters = getRightPositionMeters();

    wheelPositions = new DifferentialDriveWheelPositions(leftEncoderL.getPosition(), rightEncoderL.getPosition());

    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), new Rotation2d(), wheelPositions);
    pose = poseEstimator.getEstimatedPosition();
  }

  public double getLeftPositionMeters() {
    return leftEncoderL.getPosition() * Constants.DriveConstants.WHEEL_DIAMETER_METERS;
  }
  public double getRightPositionMeters() {
    return rightEncoderL.getPosition() * Constants.DriveConstants.WHEEL_DIAMETER_METERS;
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
    leftDriveL.set(volt);
    
  }
  public void setRightMotors (double volt) {
    rightDriveL.set(volt);
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
    return new DifferentialDriveWheelSpeeds(leftDriveL.getEncoder().getVelocity(), rightDriveL.getEncoder().getVelocity());
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