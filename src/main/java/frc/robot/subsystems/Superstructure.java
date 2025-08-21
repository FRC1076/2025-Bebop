// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.SuperstructureConstants.MechanismState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import lib.utils.MathHelpers;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The Superstructure class contains all of subsystems and commands for the robot's Superstructure.
 * This allows all of the subsystems to talk to each other,
 * allows the sensors to interact with subystems, and 
 * contains command factories for actions requiring multiple subsystems.
 */
public class Superstructure {
    /** A mutable (you can change the state after instantiation) class representing the Superstructure's desired state. */
    public static class MutableSuperState {
        protected MechanismState mechanismState;
        protected BooleanSupplier hasNote;

        public MutableSuperState(MechanismState initialState, BooleanSupplier hasNoteSupplier) {
            this.mechanismState = initialState;
            this.hasNote = hasNoteSupplier;
        }

        public MutableSuperState(BooleanSupplier hasNoteSupplier) {
            this.mechanismState = MechanismState.HOME;
            this.hasNote = hasNoteSupplier;
        }

        public void setMechanismState(MechanismState state) {
            this.mechanismState = state;
        }

        public MechanismState getMechanismState() {
            return this.mechanismState;
        }

        @AutoLogOutput(key="SuperState/MechanismState")
        public String getMechanismStateString() {
            return this.mechanismState.toString();
        }

        @AutoLogOutput(key="SuperState/HasNote")
        public boolean hasNote() {
            return this.hasNote.getAsBoolean();
        }
    }
    
    private final ArmSubsystem m_arm;
    private final IndexSubsystem m_index;
    private final IntakeSubsystem m_intake;
    private final ShooterSubsystem m_shooter;

    private final MutableSuperState superState;

    public final SuperstructureCommandFactory commandBuilder;

    public Superstructure(
        ArmSubsystem arm,
        IndexSubsystem index,
        IntakeSubsystem intake,
        ShooterSubsystem shooter,
        BooleanSupplier hasNoteSupplier
    ) {
        this.m_arm = arm;
        this.m_index = index;
        this.m_intake = intake;
        this.m_shooter = shooter;

        this.superState = new MutableSuperState(hasNoteSupplier);

        this.commandBuilder = new SuperstructureCommandFactory(this);
    }

    public ArmSubsystem getArmSubsystem() {
        return m_arm;
    }

    public IndexSubsystem getIndexSubsystem() {
        return m_index;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return m_intake;
    }
    
    public ShooterSubsystem getShooterSubsystem() {
        return m_shooter;
    }
    
    public MutableSuperState gMutableSuperState() {
        return this.superState;
    }

    public SuperstructureCommandFactory getCommandbuilder() {
        return this.commandBuilder;
    }

    /** Sets the state of the robot based on the arm's position and whether we have a note */
    public void detectState() {
        if (!superState.hasNote()) {
            superState.setMechanismState(MechanismState.HOME);
        } else if (m_arm.getPosition() < MathHelpers.getAverage(
            MechanismState.SUBWOOFER.armPositionRadians,
            MechanismState.MID_LOW.armPositionRadians)
        ) {
            superState.setMechanismState(MechanismState.SUBWOOFER);
        } else if (m_arm.getPosition() < MathHelpers.getAverage(
            MechanismState.MID_LOW.armPositionRadians,
            MechanismState.MID_HIGH.armPositionRadians)
        ) {
            superState.setMechanismState(MechanismState.MID_LOW);
        } else if (m_arm.getPosition() < MathHelpers.getAverage(
            MechanismState.MID_HIGH.armPositionRadians,
            MechanismState.AMP.armPositionRadians)
        ) {
            superState.setMechanismState(MechanismState.MID_HIGH);
        } else {
            superState.setMechanismState(MechanismState.AMP);
        }
    }

    /**
     * Apply a state to the mechanisms, with the arm achieving the desired position
     * before any of the other subsystems have their state set. 
     * 
     * @param state Desired state of the robot
     * @return A Command that applies the desired state
     */
    private Command applyStateArmFirst(MechanismState state) {
        superState.setMechanismState(state);

        return Commands.sequence(
            m_arm.startPid(state.armPositionRadians),
            Commands.waitUntil(() -> m_arm.withinTolerance(ArmConstants.kToleranceRadians)),
            Commands.parallel(
                m_intake.runVolts(state.intakeVolts),
                m_index.runVolts(state.indexVolts),
                m_shooter.startPid(state.shooterLeftSpeedRadPerSec, state.shooterRightSpeedRadPerSec)
            )
        ); 
    }

    /**
     * Apply the desired state to of the subsystems at once,
     * 
     * @param state The desired state of the robot
     * @return A Command that applies the desired state
     */
    private Command applyStateAllParallel(MechanismState state) {
        superState.setMechanismState(state);

        return Commands.parallel(
            m_arm.startPid(state.armPositionRadians),
            m_intake.runVolts(state.intakeVolts),
            m_index.runVolts(state.indexVolts),
            m_shooter.startPid(state.shooterLeftSpeedRadPerSec, state.shooterRightSpeedRadPerSec)
        );
    }

    /**
     * Apply the desired state to all the subsystems on the robot, except for the arm.
     * 
     * @param state The desired state of the robot
     * @return A Command that applies the desired state
     */
    private Command applyStateNoArmMove(MechanismState state) {
        superState.setMechanismState(state);

        return Commands.parallel(
            m_intake.runVolts(state.intakeVolts),
            m_index.runVolts(state.indexVolts),
            m_shooter.startPid(state.shooterLeftSpeedRadPerSec, state.shooterRightSpeedRadPerSec)
        );
    }

    /** Contains all of the command factories for the Superstructure
     * (and all of the commands that use those command factories).
     */
    public class SuperstructureCommandFactory {
        private final Superstructure superstructure;
        private final Map<MechanismState, MechanismState> scoringStates = new HashMap<MechanismState, MechanismState>();

        private SuperstructureCommandFactory(Superstructure superstructure) {
            this.superstructure = superstructure;

            scoringStates.put(MechanismState.SUBWOOFER, MechanismState.SHOOT_SUBWOOFER);
            scoringStates.put(MechanismState.MID_LOW, MechanismState.SHOOT_MID_LOW);
            scoringStates.put(MechanismState.MID_HIGH, MechanismState.SHOOT_MID_HIGH);
            scoringStates.put(MechanismState.AMP, MechanismState.SHOOT_AMP);
        }

        /** Command to go to the home state */
        public Command home() {
            return applyStateAllParallel(MechanismState.HOME);
        }
        
        /** Command to go to the intake state.
         *  Once a note is obtained, automatically goes to the subwoofer state.
         */
        public Command intake() {
            return Commands.sequence(
                applyStateArmFirst(MechanismState.INTAKE),
                Commands.waitUntil(superstructure.superState.hasNote),
                applyStateAllParallel(MechanismState.SUBWOOFER)
            );
        }

        /** Command to go to the subwoofer pre-scoring state */
        public Command subwoofer() {
            return applyStateAllParallel(MechanismState.SUBWOOFER);
        }

        /** Command to go to the mid-low pre-scoring state */
        public Command midLow() {
            return applyStateAllParallel(MechanismState.MID_LOW);
        }
        /** Command to go to the mid-high pre-scoring state */
        public Command midHigh() {
            return applyStateAllParallel(MechanismState.MID_HIGH);
        }
        
        /** Command to go to the amp pre-scoring state */
        public Command amp() {
            return applyStateAllParallel(MechanismState.AMP);
        }

        /** Command to shoot the note based on the pre-scoring state.
         *  Once the note has been shot, returns to the home state.
         */
        public Command shoot() {
            return Commands.sequence(
                applyStateNoArmMove(
                    scoringStates.getOrDefault(superstructure.superState.getMechanismState(), MechanismState.SHOOT_MID_HIGH)
                ),
                Commands.waitUntil(
                    new Trigger(superstructure.superState.hasNote)
                        .debounce(SuperstructureConstants.kArmMoveDebounceTimeAfterShoot)
                ),
                applyStateAllParallel(MechanismState.HOME)
            );
        }

        /** Detect the mechanism state based on the arm's position after manual control */
        public Command detectMechanismState() {
            return Commands.runOnce(() -> superstructure.detectState());
        }

        public Command forceForward() {
            return Commands.parallel(
                m_intake.runVolts(SuperstructureConstants.kIntakeManualControlVoltage),
                m_index.runVolts(SuperstructureConstants.kIndexManualControlVoltage),
                Commands.run(
                    () -> m_shooter.setVoltage(
                        SuperstructureConstants.kShooterLeftManualControlVoltage,
                        SuperstructureConstants.kShooterRightManualControlVoltage
                    ),
                    m_shooter
                )
            );
        }
        
        public Command forceBackward() {
            return Commands.parallel(
                m_intake.runVolts(SuperstructureConstants.kIntakeManualControlVoltage * -1),
                m_index.runVolts(SuperstructureConstants.kIndexManualControlVoltage * -1),
                Commands.run(
                    () -> m_shooter.setVoltage(
                        SuperstructureConstants.kShooterLeftManualControlVoltage * -1,
                        SuperstructureConstants.kShooterRightManualControlVoltage * -1
                    ),
                    m_shooter
                )
            );
        }

        public Command armUpManual() {
            return Commands.run(
                () -> m_arm.setVoltage(SuperstructureConstants.kArmManualControlVoltage),
                m_arm
            );
        }
        
        public Command armDownManual() {
            return Commands.run(
                () -> m_arm.setVoltage(SuperstructureConstants.kArmManualControlVoltage * -1),
                m_arm
            );
        }
    }
} 
    