/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Trajectory;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TrajectoryFactory extends SubsystemBase {
  AHRS ahrs = new AHRS(SPI.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.7407);      // 輪子寬度
  DifferentialDriveOdometry odmetry = new DifferentialDriveOdometry(getHeading());

  Pose2d pose;

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.143, 2.23, 0.372);

  PIDController lpidcontroller = new PIDController(
      1.5, 0, 0);
  PIDController rpidcontroller = new PIDController(
      1.5, 0, 0);

  /**
   * Creates a new TrajectoryFactory.
   */
  public TrajectoryFactory() {

  }
  public void reset(){
    ahrs.reset();
    RobotContainer.drivetrain.reset();
  }
  /**
   * set odmetry,let the starting position 
   * of the robot be the starting point of the trajectory
   * 
   * @param pose2d the trajectory origin
   */
  public void setOdmetry(Pose2d pose2d){
    odmetry.resetPosition(pose2d, pose2d.getRotation());
  }

  /**
   * Packaging the gyroscope's angle into an Rotation2d object
   * 
   * @return the heading, clockwise is negative
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-ahrs.getAngle());
  }

  /**
   * encoder velocity to chassis speed
   * 
   * @return current chassis speed
   */
  public DifferentialDriveWheelSpeeds getSpeed() {
    SmartDashboard.putNumber("leftRate", RobotContainer.drivetrain.getLeftVelocity());
    SmartDashboard.putNumber("rightRate", RobotContainer.drivetrain.getRightVelocity());
    return new DifferentialDriveWheelSpeeds(
      RobotContainer.drivetrain.getLeftVelocity(),
      RobotContainer.drivetrain.getRightVelocity()
      );
  }

  /**
   * Provide feedforward controller
   * 
   * @return feedForward controlller 
   */
  public SimpleMotorFeedforward getFeedforward() {
    return feedForward;
  }

  /**
   * Provide PID controller
   * 
   * @return left PID controller
   */
  public PIDController getlpidcontroller() {
    return lpidcontroller;
  }

  /**
   * Provide PID controller
   * 
   * @return right PID controller
   */
  public PIDController getrpidcontroller() {
    return rpidcontroller;
  }

  /**
   * Provide kinematics object, contain track width
   * 
   * @return kinematics
   */
  public  DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Provide current pose, update from Periodic
   * 
   * @return current pose
   */
  public Pose2d getpose2d(){
    return pose;
  }
  /**
   * get "X" from odmetry
   * 
   * @return current "X"
   */
  public double getX(){
    return odmetry.getPoseMeters().getTranslation().getX();
  }
  /**
   * get "Y" from odmetry
   * 
   * @return current "Y"
   */
  public double getY(){
    return odmetry.getPoseMeters().getTranslation().getY();
  }
  public Command getRamseteCommand(Trajectory trajectory){
    RamseteCommand command = new RamseteCommand(
            trajectory,
            Robot.trajectoryFactory::getpose2d, 
            new RamseteController(2.0, 0.7),
            Robot.trajectoryFactory.getFeedforward(), 
            Robot.trajectoryFactory.getKinematics(), 
            Robot.trajectoryFactory::getSpeed, 
            Robot.trajectoryFactory.getlpidcontroller(),
            Robot.trajectoryFactory.getrpidcontroller(), 
            RobotContainer.drivetrain::setOutput,
            Robot.trajectoryFactory
            );
            return command;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
