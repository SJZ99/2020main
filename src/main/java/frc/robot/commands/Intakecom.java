/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Powercell.Arm;
import frc.robot.subsystems.Powercell.Intake;

public class Intakecom extends CommandBase {
  Arm m_arm;
  Intake m_intake;
  double joystick =0;
  /**
   * Creates a new Intake.
   */
  public Intakecom(Arm arm, Intake intake,double va) {
    m_arm = arm;
    m_intake = intake;
    joystick = va;
    addRequirements(m_arm);
    addRequirements(m_intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick<-0.5){
    m_arm.armdown();
    m_intake.intake();}
    else if(joystick>0.5){
      m_arm.armup();
      m_intake.intakestop();
      SmartDashboard.putNumber("intake", 1);
      }
    else{
      m_arm.armstop();
    }
    }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.armup();
    m_intake.intakestop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
