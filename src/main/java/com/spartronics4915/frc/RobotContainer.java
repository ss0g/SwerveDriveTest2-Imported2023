// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc;

import com.spartronics4915.frc.commands.Autos;
import com.spartronics4915.frc.commands.DebugTeleopCommands;
import com.spartronics4915.frc.commands.SwerveCommands;
import com.spartronics4915.frc.commands.SwerveTrajectoryFollowerCommands;
import com.spartronics4915.frc.subsystems.Swerve;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static com.spartronics4915.frc.Constants.OI.*;

/**
* This class is where the bulk of the robot should be declared. Since
* Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in
* the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of
* the robot (including
* subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
    private final XboxController mController;
    
    private final SwerveTrajectoryFollowerCommands mSwerveTrajectoryFollowerCommands;
    
    // The robot's subsystems and commands are defined here...
    private final Swerve mSwerve;
    private final SwerveCommands mSwerveCommands;
    
    private final Autos mAutos;
    
    private final Command mAutonomousCommand;
    
    private final boolean useJoystick = false;
    // private final Command mTestingCommand;
    
    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer() {
        if (useJoystick)
        mController = new XboxController(kControllerID);
        else
        mController = null;
        
        mSwerve = new Swerve();
        mSwerveCommands = new SwerveCommands(mController, mSwerve);
        
        mSwerveTrajectoryFollowerCommands = new SwerveTrajectoryFollowerCommands(mSwerve);
        
        mAutos = new Autos(mSwerve, mSwerveTrajectoryFollowerCommands);
        
        // mAutonomousCommand = mAutos.new MoveForwardCommand();
        mAutonomousCommand = mAutos.new MoveForwardCommandFancy();
        // mTeleopInitCommand = null;// DebugTeleopCommands.getShuffleboardInitCommand(mSwerve);
        //mTeleopCommand = mSwerveCommands.new TeleopCommand();
        // mTestingCommand = mSwerveCommands.new TestCommand();
        
        // Configure the button bindings
        configureButtonBindings();
    }
    
    /**
    * Use this method to define your button->command mappings. Buttons can be
    * created by
    * instantiating a {@link GenericHID} or one of its subclasses ({@link
        * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
        * it to a {@link
            * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
            */
            private void configureButtonBindings() {
                if(useJoystick) {
                    new JoystickButton(mController, kToggleFieldRelativeButton)
                    .onTrue(mSwerveCommands.new ToggleFieldRelative());
                    
                    new JoystickButton(mController, kResetYawButton)
                    .onTrue(mSwerveCommands.new ResetYaw());
                    
                    new JoystickButton(mController, kResetOdometryButton)
                    .onTrue(mSwerveCommands.new ResetOdometry());
                }
            }
            
            /**
            * Use this to pass the autonomous command to the main {@link Robot} class.
            *
            * @return the command to run in autonomous
            */
            public Command getAutonomousCommand() {
                return mAutonomousCommand;
            }
            
            public Command getTeleopCommand() {
                return null;
            }
            
            public Command getTestingCommand() {
                // return mTestingCommand;
                return null;
            }

            public Command getRobotInitCommand() {

                return new DebugTeleopCommands.ShuffleboardUpdateCommand(mSwerve);
            }
            public void initTeleop() {
                DebugTeleopCommands.TeleopInit(mSwerve);
            }
        }
        