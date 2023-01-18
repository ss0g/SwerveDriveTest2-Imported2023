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
        private GenericEntry state_angle;
        SwerveModuleWidget(ShuffleboardTab tab, String name) {
            ShuffleboardLayout swerve_module = tab.getLayout(name, BuiltInLayouts.kList)
            .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));
    
            angleEntry = swerve_module.add("desired.angle", 0).getEntry();
            state_angle = swerve_module.add("current.angle", 0).getEntry();
        }
        
        public void update(SwerveModuleState current, SwerveModuleState desired) {
            angleEntry.setDouble(desired.angle.getDegrees()); 
            state_angle.setDouble(current.angle.getDegrees()); 
        }
    }
    public static class SwerveTab {
        SwerveModuleWidget module0, module1, module2, module3;
        ShuffleboardTab tab;
        Swerve swerve_subsystem;

        SwerveTab(Swerve swerve) {
            tab = Shuffleboard.getTab("Swerve");
            module0 = new SwerveModuleWidget(tab, "Module 0");
            module1 = new SwerveModuleWidget(tab, "Module 1");
            module2 = new SwerveModuleWidget(tab, "Module 2");
            module3 = new SwerveModuleWidget(tab, "Module 3");
            swerve_subsystem = swerve;
            ShuffleboardLayout elevatorCommands = 
            tab.getLayout("Orientation", BuiltInLayouts.kList)
            .withSize(2, 2)
            .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
    
            elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(0)).withName("Orientation 0"));
            elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem, Rotation2d.fromDegrees(90)).withName("Orientation 90"));
    

        }

    public void update(){

        var swerve_module_desired_states = swerve_subsystem.getDesiredStates();
        var swerve_module_states = swerve_subsystem.getStates();

        module0.update(swerve_module_states[0], swerve_module_desired_states[0]);
        module1.update(swerve_module_states[1], swerve_module_desired_states[1]);
        module2.update(swerve_module_states[2], swerve_module_desired_states[2]);
        module3.update(swerve_module_states[3], swerve_module_desired_states[3]);
    }
}





    public static CommandBase getShuffleboardInitCommand(Swerve swerve_subsystem) {

        SwerveTab tab  = new SwerveTab(swerve_subsystem);
        return Commands.run(()->tab.update());
    }
}
