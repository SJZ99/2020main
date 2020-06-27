




/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Aim;
import frc.robot.commands.FastShoot;
import frc.robot.commands.Intakecom;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Powercell.Aimer;
import frc.robot.subsystems.Powercell.Arm;
import frc.robot.subsystems.Powercell.Intake;
import frc.robot.subsystems.Powercell.Shooter;
import frc.robot.subsystems.Powercell.Turret;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Easyauto extends SequentialCommandGroup {
  /**
   * Creates a new Easyauto.
   */
  public Easyauto(Turret turret,Drivetrain drivetrain,Vision vision,Intake intake,Shooter shooter,Aimer aimer,Arm arm) {


    super(
      new ParaShot(turret,vision,aimer,shooter),
      new RunCommand(()->drivetrain.curvaturedrive(0.2, 0, false),drivetrain).withTimeout(1)
      //new InstantCommand(()->intake.intake(),intake),
    ///  new InstantCommand(()->arm.armdown(),arm),
     /// new WaitCommand(2).andThen(
        //new InstantCommand(()->drivetrain.curvaturedrive(0, 0, false))),
      //new ParaShot(turret,vision,aimer,shooter)
    );
    
  }
}
