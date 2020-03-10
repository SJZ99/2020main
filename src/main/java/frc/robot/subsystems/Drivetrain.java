/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DrCon;
import frc.robot.Setmotor;

public class Drivetrain extends SubsystemBase {
  

  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DrCon.wheelPitch); // 輪子寬度
  private DifferentialDriveOdometry odmetry = new DifferentialDriveOdometry(getHeading());
  private SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 40, 50, 1);
  private Setmotor setmotor = new Setmotor();
  
  private Pose2d pose = new Pose2d();
  private WPI_TalonFX leftmas = new WPI_TalonFX(DrCon.LeftmasterID);
  private WPI_TalonFX leftfol= new WPI_TalonFX(DrCon.LeftfollowerID);
  private WPI_TalonFX rightmas = new WPI_TalonFX(DrCon.RightmasterID);
  private WPI_TalonFX rightfol = new WPI_TalonFX(DrCon.RightfollowerID);
  
 // private AHRS ahrs;

  private AHRS ahrs = new AHRS(SPI.Port.kMXP);
  private double disttar,a=0.3,angle=0;
  private double m_quickStopAccumulator = 0,leftout=0,rightout=0;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    
    leftmas.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    rightmas.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    leftmas.setSelectedSensorPosition(0,0, 10);
    rightmas.setSelectedSensorPosition(0,0, 10);
    setmotor.setmotor(leftmas, supplyCurrentLimitConfiguration, DrCon.kP, DrCon.kF, InvertType.None, DrCon.pidsolt, DrCon.Ramptime, DrCon.timeoutMs);
    setmotor.setmotor(rightmas, supplyCurrentLimitConfiguration, DrCon.kP, DrCon.kF, InvertType.InvertMotorOutput, DrCon.pidsolt, DrCon.Ramptime,DrCon.timeoutMs);
    setmotor.setmotorfol(leftfol, supplyCurrentLimitConfiguration, InvertType.FollowMaster,DrCon.timeoutMs);
    setmotor.setmotorfol(rightfol, supplyCurrentLimitConfiguration, InvertType.FollowMaster,DrCon.timeoutMs);
    leftfol.follow(leftmas);
    rightfol.follow(rightmas);
    leftmas.configMaxIntegralAccumulator(0, 5000);
    rightmas.configMaxIntegralAccumulator(0, 5000);
    leftmas.setSensorPhase(false);
    rightmas.setSensorPhase(true);
    leftmas.configMotionCruiseVelocity(10000, 10);
    rightmas.configMotionCruiseVelocity(10000, 10);
    
		leftmas.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);
    leftmas.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
    rightmas.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);
    rightmas.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
    
     //跟隨降低頻率
    leftfol.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,255);
    leftfol.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,255);
       
    rightfol.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,255);
    rightfol.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,255);
    

  }
  /**
   * 
   * @param dist meter
   */
  public void drivedist(double dist){
    leftmas.set(TalonFXControlMode.MotionMagic,dist*DrCon.enoderunit);

  }
 
  
  public void reset() {
    ahrs.reset();
  }

  /**
   * set odmetry,let the starting position of the robot be the starting point of
   * the trajectory
   * 
   * @param pose2d the trajectory origin
   */
  public void setOdmetry(Pose2d pose2d) {
    odmetry.resetPosition(pose2d, pose2d.getRotation());
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(angle);

    // return Rotation2d.fromDegrees(-ahrs.getAngle());
  }

  public void Trainit(String path) throws IOException {
    reset();
    setOdmetry(TrajectoryMaker.getTrajectory(path).getInitialPose());
    SmartDashboard.putNumber("TotalTime", TrajectoryMaker.getTrajectory(path).getTotalTimeSeconds());
    
  }
  
    /**
     * set motor output,
     * 
     * @param left    left velocity
     * @param right   right velocity
     */
    public void setOutput(double left, double right) {
      leftmas.set(ControlMode.Velocity, left / DrCon.distantsPerPulse);
      rightmas.set(ControlMode.Velocity, right / DrCon.distantsPerPulse);
      // leftmotor.set(ControlMode.PercentOutput, 0);
      // rightmotor.set(ControlMode.PercentOutput, 0);
      SmartDashboard.putNumber("leftOutput ", left / DrCon.distantsPerPulse);
      SmartDashboard.putNumber("rightOutput", right / DrCon.distantsPerPulse);
    }
  
  public DifferentialDriveWheelSpeeds getSpeed() {
    SmartDashboard.putNumber("leftRate", leftmas.getSelectedSensorVelocity() * DrCon.distantsPerPulse);
    SmartDashboard.putNumber("rightRate", rightmas.getSelectedSensorVelocity() * DrCon.distantsPerPulse);
    return new DifferentialDriveWheelSpeeds(
      leftmas.getSelectedSensorVelocity() * DrCon.distantsPerPulse, 
      rightmas.getSelectedSensorVelocity() * DrCon.distantsPerPulse
      );
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
  public void setmotor(){
    rightmas.setSelectedSensorPosition(0);
    leftmas.setSelectedSensorPosition(0);

    rightmas.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0 ,0);
    leftmas.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0 ,0);
    rightfol.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0 ,0);
    leftfol.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0 ,0);

    rightmas.configVoltageCompSaturation(10);
    leftmas.configVoltageCompSaturation(10);
    rightmas.enableVoltageCompensation(true);
    leftmas.enableVoltageCompensation(true);
  }

  public void message(){
    SmartDashboard.putNumber("x", getX());
    SmartDashboard.putNumber("Y", getY());
    //distants
    
    SmartDashboard.putNumber("leftDistants", leftmas.getSelectedSensorPosition() * DrCon.distantsPerPulse);
    SmartDashboard.putNumber("rightDistants", rightmas.getSelectedSensorPosition() * DrCon.distantsPerPulse);
    //SmartDashboard.putNumber("Yaw", ahrs.getYaw());
  }
  public boolean drivedistend(){
    return Math.abs(disttar-leftmas.getSelectedSensorPosition(0))<500;
  }
  public void curvaturedrive(double xSpeed,double zRotation,boolean isQuickTurn){
    
    curvatureDrive(0.75*xSpeed,zRotation, isQuickTurn);
    rightmas.set(ControlMode.PercentOutput, rightout);
    leftmas.set(ControlMode.PercentOutput,leftout);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  /**
   * Curvature drive method for differential drive platform.
   *
   * <p>The rotation argument controls the curvature of the robot's path rather than its rate of
   * heading change. This makes the robot more controllable at high speeds. Also handles the
   * robot's quick turn functionality - "quick turn" overrides constant-curvature turning for
   * turn-in-place maneuvers.
   *
   * @param xSpeed      The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation   The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                    positive.
   * @param isQuickTurn If set, overrides constant-curvature turning for
   *                    turn-in-place maneuvers.
   */
  @SuppressWarnings({"ParameterName", "PMD.CyclomaticgitComplexity"})
  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    if(Math.abs(zRotation)<0.05){
      zRotation =0;
    }
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    double angularPower;
    boolean overPower;
    double m_quickStopAlpha =0.1;
    if (isQuickTurn) {
      if (Math.abs(xSpeed) < 0.1) {
        m_quickStopAccumulator = (1 - 0.1) * m_quickStopAccumulator
            + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
      }
      
      overPower = true;
      angularPower = zRotation;
    } else {
      overPower = false;
      angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) {
        m_quickStopAccumulator -= 1;
      } else if (m_quickStopAccumulator < -1) {
        m_quickStopAccumulator += 1;
      } else {
        m_quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = xSpeed + angularPower;
    double rightMotorOutput = xSpeed - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }

    leftout=  leftMotorOutput;
    rightout = rightMotorOutput;
    
  }
}