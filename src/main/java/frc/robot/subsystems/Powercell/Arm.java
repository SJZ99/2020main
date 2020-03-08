/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Powercell;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PowCon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase {
  private WPI_VictorSPX arm = new WPI_VictorSPX(PowCon.intakearmmasID);
  private DigitalInput forwardlimit = new DigitalInput(PowCon.forlimitID);
  private DigitalInput backwardlimit = new DigitalInput(PowCon.backlimitID);
  String status ="MID";
  /**
   * Creates a new Arm.
   */

  public Arm() {
    arm.setInverted(false);
  }
  public void armdown(){
   // if(getarmstatus()!="Down"){
    arm.set(ControlMode.PercentOutput,0.6);
  //}
  }
  public void armup(){
    //if(getarmstatus()!="UP"){
    arm.set(ControlMode.PercentOutput,-0.6);
  //}
    
  }
  public void armstop(){
    arm.set(ControlMode.PercentOutput,0.0);
  }
  public String getarmstatus(){
    if(forwardlimit.get()&&!backwardlimit.get()){
    status ="UP" ;
  }
    else if(forwardlimit.get()){
      status = "Down";
    }
    else if(!forwardlimit.get()&!backwardlimit.get()){
      status = "MID";
    }
    else{
      status = "ERR";

    }
    return status;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("ArmStatus", status);
    // This method will be called once per scheduler run
  }
}
