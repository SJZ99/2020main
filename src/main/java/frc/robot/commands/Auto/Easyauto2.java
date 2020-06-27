
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.Aim;
import frc.robot.commands.FastShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Powercell.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Easyauto2 extends SequentialCommandGroup {
  /**
   * Creates a new Easyauto.
   */
  public Easyauto2(Shooter shooter,Drivetrain drivetrain,Vision vision) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    
    super(
      new InstantCommand(()->SmartDashboard.putString("Auto Mode", "EasyAuto2")),
      new FastShoot(shooter),
      new StartEndCommand(()->drivetrain.drivedist(1),()->drivetrain.drivedist(0),
      drivetrain).withInterrupt(() -> drivetrain.drivedistend()));
    
  }
}