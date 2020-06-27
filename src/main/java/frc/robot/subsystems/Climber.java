/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CliCon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {
  private WPI_VictorSPX climbermas = new WPI_VictorSPX(CliCon.climbermasID);
  private WPI_VictorSPX climberfol = new WPI_VictorSPX(CliCon.climberfolID);
  private Solenoid hooksSolenoid = new Solenoid(CliCon.hookID);
  private DoubleSolenoid riseSolenoid = new DoubleSolenoid(CliCon.riseID[0],CliCon.riseID[1]);
  private Compressor compressor = new Compressor();
  private String climstatus = "unRised";
  //Hookup 鉤子伸起
  //Rised 舉起汽缸 鉤子沒伸
  //unRise 汽缸未舉起
  //climbing 爬升 

  /**
   * Creates a new Climber.
   */
  public Climber() {
    compressor.start();
    compressor.setClosedLoopControl(true);
    climbermas.configFactoryDefault();
    climberfol.configFactoryDefault();
    climberfol.follow(climbermas);
    climberfol.setInverted(InvertType.OpposeMaster);
  }

  public void rise(){
    if(climstatus == "unRise"){
    riseSolenoid.set(Value.kForward);
    climstatus = "Rised";
    }
  }
  public void down(){
    if(climstatus=="Rised"){
    riseSolenoid.set(Value.kReverse);
    climstatus = "unRise";
    }
  }
  public void hookup(){
    if(climstatus == "Rised"|climstatus=="Hookup"){
    hooksSolenoid.set(true);
    climstatus = "Hookup";
    }

  }
  public void hookdown(){
    hooksSolenoid.set(false);
  }
  public void botclimb(){
    if(climstatus == "Hookup"){
    climbermas.set(ControlMode.PercentOutput,0.5);
    //untest
    }
  }
  
  public void botstopclimb(){
    climbermas.set(ControlMode.PercentOutput,0.0);
    //untest
    }
  





  @Override
  public void periodic() {
    SmartDashboard.putData(compressor);

    // This method will be called once per scheduler run
  }
}
