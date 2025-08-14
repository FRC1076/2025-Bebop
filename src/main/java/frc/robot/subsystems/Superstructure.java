package frc.robot.subsystems;

import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.SuperstructureConstants.MechanismState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

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

        @AutoLogOutput(key="SuperState/MechanismState")
        public MechanismState getMechanismState() {
            return this.mechanismState;
        }

        @AutoLogOutput(key="SuperState/HassNote")
        public boolean hasNote() {
            return this.hasNote.getAsBoolean();
        }
    }
    
    private final ArmSubsystem m_arm;
    private final IndexSubsystem m_index;
    private final IntakeSubsystem m_intake;
    private final ShooterSubsystem m_shooter;

    private final MutableSuperState superState;

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

    /** Contains all of the command factories for the Superstructure
     * (and all of the commands that use those command factories).
     */
    public class SuperstructureCommandFactory {
        private final Superstructure superstructure;

        private SuperstructureCommandFactory(Superstructure superstructure) {
            this.superstructure = superstructure;
        }
    }
} 
     