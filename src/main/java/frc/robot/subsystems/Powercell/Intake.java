/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Powercell;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PowCon;

public class Intake extends SubsystemBase {
  private WPI_TalonSRX intake = new WPI_TalonSRX(PowCon.intakeID);
  private SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 30, 35, 1);
  /**
   * Creates a new Intake.
   */
  public Intake() {
    
    intake.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
  }
  public  void intake() {
    intake.set(ControlMode.PercentOutput,1);
  }
  public  void intakestop() {
    intake.set(ControlMode.PercentOutput,0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
