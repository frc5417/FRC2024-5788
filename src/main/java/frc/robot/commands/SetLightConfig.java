// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// // import frc.robot.subsystems.LightsControl;
// import frc.robot.subsystems.LightsControl;

// public class SetLightConfig extends Command {
//   // private final LightsControl lightsControl;
//   // private int configNum;

//   private final LightsControl newLightsControl;
//   private boolean configStatus;

//   /** Creates a new SetLightConfig. */
//   public SetLightConfig(LightsControl subsystem, boolean configureStatus) {
//     this.newLightsControl = subsystem;
//     this.configStatus = configureStatus;

//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(newLightsControl);
//   }

// //   public SetLightConfig(LightsControl mLightscontrol, int i) {
// //     this.lightsControl = new LightsControl();
// //     //TODO Auto-generated constructor stub
// // }

// // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     // lightsControl.setLightConfig(configNum);
//     LightsControl.configLights(configStatus);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }