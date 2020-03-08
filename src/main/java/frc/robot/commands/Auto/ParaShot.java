/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Powercell.Aimer;
import frc.robot.subsystems.Powercell.Shooter;
import frc.robot.subsystems.Powercell.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ParaShot extends ParallelCommandGroup {
  /**
   * Creates a new ParaShot.
   */
  public ParaShot(Turret turret,Vision vision,Aimer aimer,Shooter shooter) {

    super(new AutoAim(turret, vision, aimer).
    andThen(new AutoShot(shooter)).withTimeout(3),
    new InstantCommand(()->shooter.flywheelspinup(12000),shooter));


    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
  }
}
