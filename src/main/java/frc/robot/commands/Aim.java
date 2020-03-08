/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Powercell.Turret;


public class Aim extends CommandBase {
  private Turret turretsub;
  private Vision visionsub;
  //private Aimer  aimersub;
  double x;
  /**
   * Creates a new Aim.
   */
  public Aim(Turret turret,Vision vision) {
    turretsub = turret;
    visionsub = vision;
    //aimersub  = aimer;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretsub);
    addRequirements(visionsub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // SmartDashboard.putNumber("ㄏ", 22);
  //  drivetrainsub.di  staim(visionsub.getDisterr());
 // 
    x=visionsub.getx();
    SmartDashboard.putNumber("getx", x);
    turretsub.turretaim(x);
    
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   turretsub.turretaim(0);
   // drivetrainsub.distaim(0);

  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    return false;
  }
}
