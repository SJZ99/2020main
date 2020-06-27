/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Powercell;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PowCon;

public class Aimer extends SubsystemBase {
  WPI_TalonSRX aimer = new WPI_TalonSRX(PowCon.aimerID);
  double unit = 0,Dist;
  //單位正，向後
  Joystick joystick = new Joystick(0);  
  
  /**
   * Creates a new Aimer.
   */
  public Aimer() { 
		aimer.setSensorPhase(true);
    aimer.setInverted(false);
    aimer.configFactoryDefault();
    aimer.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,10);
    aimer.config_kF(0,PowCon.aimerkF);
    aimer.config_kP(0,PowCon.aimerkP);
    aimer.config_kI(0, PowCon.aimerkI);
    aimer.config_kD(0, PowCon.aimerID);
    aimer.config_IntegralZone(0, PowCon.aimerizone);
    
		aimer.configNeutralDeadband(PowCon.deadband, 10);
    aimer.configMotionAcceleration(1600, 10);
    aimer.configMotionCruiseVelocity(1500,10);
    aimer.setSelectedSensorPosition(0);
    aimer.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);
    aimer.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
    aimer.selectProfileSlot(0, 0);
    //test
    SmartDashboard.putNumber("finish", 1);

  }
  public void aimunit(){
    
    aimer.set(ControlMode.MotionMagic,-unit);
    
    
  }
  public boolean isAimfinish(){
    SmartDashboard.putNumber("err", aimer.getClosedLoopError(0));
    return Math.abs(aimer.getClosedLoopError(0))<50;

  }

  public void resetAimer(){
    aimer.setSelectedSensorPosition(0, 0,10);
  }
  public void Uplimit(){
    aimer.setSelectedSensorPosition(-10000);
  }
  public void Downlimit(){
    aimer.setSelectedSensorPosition(0);
  }

  public double getunit(){
    if (Dist>600){
      unit= 11200+100*(Dist-600/50);
    }else if(Dist>500){
      unit=11000+200*(Dist-500)/50;
    }
    else if(Dist>450){
      unit =8600-1100*(Dist-450)/50;
    }
    else if(Dist>400){
      unit=8450+150*(Dist-400)/50;
    }
    else if(Dist>350){
      unit =8000+450*(Dist-350)/50;
    }
    else if(Dist>300){
      unit = 8050-50*(Dist-300)/50;
    }
    else if(Dist>250){
      unit = 7300+750*(Dist-250)/50;

    }
    else if(Dist>200){
      unit = 6100+1200*(Dist-200)/50;

    }
    else if(Dist>150){
      unit = 5200+900*(Dist-150)/50;
    }
    else if(Dist>100){
      unit=941+4260*(Dist-100)/50;
    }
    else{
      unit =0;

    }
    
    return unit;

  }
  
  

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Dist", Dist);
   // SmartDashboard.putNumber("Vel", aimer.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Pos", aimer.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("err", aimer.getClosedLoopError(0));
   // SmartDashboard.putNumber("out", aimer.getMotorOutputPercent());
    Dist = Sensor.getlimelightDist();
    //SmartDashboard.putNumber("unit", unit);
    // This method will be called once per scheduler run
    //test
    //unit =3000*joystick.getRawAxis(3);
    //aimunit();
  }
}
