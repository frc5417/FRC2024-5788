// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class CustomNamedCommands {
    public static HashMap<String, Command> command_map = new HashMap<String, Command>();

    public static void registerCommand(String name, Command command) {
        command_map.put(name, command);
        System.out.println(name);
    }

    public static Command getCommand(String name) {
        return command_map.get(name);
    }
}
