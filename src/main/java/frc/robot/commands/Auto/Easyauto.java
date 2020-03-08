




/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DrCon;
import frc.robot.commands.Aim;
import frc.robot.commands.DistShooter;
import frc.robot.commands.Intakecom;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TrajectoryMaker;
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
        new Aim(turret, vision).withTimeout(1),
        new DistShooter(shooter).withTimeout(4),
        new InstantCommand(()->intake.intake(),intake),
        new RamseteCommand(TrajectoryMaker.getTrajectory(DrCon.LeftInitToCP),drivetrain::getpose2d, new RamseteController(2.0, 0.7), drivetrain.getKinematics(),drivetrain::setOutput, drivetrain),
        new Aim(turret, vision).withTimeout(1),
        new DistShooter(shooter).withTimeout(4),
        new InstantCommand(()->intake.intakestop(),intake)
        
        

      );
      
    }
    
  }
