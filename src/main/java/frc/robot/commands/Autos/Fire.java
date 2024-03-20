// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CustomNamedCommands;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Fire extends SequentialCommandGroup {
  /** Creates a new Fire. */
  public Fire(double waitTimeForHandoff) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      CustomNamedCommands.getCommand("ShootWrist"),
      CustomNamedCommands.getCommand("ShooterOn"),
      new WaitCommand(1),
      CustomNamedCommands.getCommand("IndexOn"),
      new WaitCommand(waitTimeForHandoff),
      CustomNamedCommands.getCommand("IndexOff"),
      CustomNamedCommands.getCommand("ShooterOff")
    );
  }
}
