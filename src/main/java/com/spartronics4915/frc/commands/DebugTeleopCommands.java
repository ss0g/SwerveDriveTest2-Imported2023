package com.spartronics4915.frc.commands;

import com.spartronics4915.frc.subsystems.Swerve;
import com.spartronics4915.frc.subsystems.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Map;

import com.spartronics4915.frc.commands.SimpleAutos;


public final class DebugTeleopCommands {

    public static class SwerveModuleWidget {
        private GenericEntry angleEntry;
        SwerveModuleWidget(ShuffleboardTab tab) {
            ShuffleboardLayout swerve_module = tab.getLayout("SwerveModule State", BuiltInLayouts.kList)
            .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));
    
            angleEntry = swerve_module.add("Angle", 0).getEntry();
        }
    }
    public static class SwerveTab {
        SwerveModuleWidget module0;
        ShuffleboardTab tab;
        Swerve swerve_subsystem;

        SwerveTab(Swerve swerve) {
            tab = Shuffleboard.getTab("Swerve");
            module0 = new SwerveModuleWidget(tab);
            swerve_subsystem = swerve;
            ShuffleboardLayout elevatorCommands = 
            tab.getLayout("Orientation", BuiltInLayouts.kList)
            .withSize(1, 2)
            .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
    
            elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(0)).withName("Orientation 0"));
            elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(90)).withName("Orientation 90"));
    

        }

    public void update(){

        var swerve_modules = swerve_subsystem.getDesiredStates();
        module0.angleEntry.setDouble(swerve_modules[0].angle.getDegrees()); 
    }
}





    public static CommandBase getShuffleboardInitCommand(Swerve swerve_subsystem) {

        SwerveTab tab  = new SwerveTab(swerve_subsystem);
        return Commands.run(()->tab.update());
    }
}
