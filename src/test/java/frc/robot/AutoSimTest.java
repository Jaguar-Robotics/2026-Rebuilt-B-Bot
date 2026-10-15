package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Handlers.FourBarHandler.FourBarState;
import frc.robot.Handlers.SuperStructure.SuperStructureState;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/** Each autonomous routine, run end to end. */
class AutoSimTest {
    @ParameterizedTest
    @ValueSource(strings = {"LB - D FeedHP", "LT - D FeedHP", "MidHub - D FeedHP"})
    void autoRunsToTheOutpostWithoutErrorsAndTheFloorStaysFlatWhileIntaking(String autoName) {
        RobotSim sim = RobotSim.get();
        sim.rc.getFuelSimulation().clearFieldForTesting();
        sim.disable();
        sim.run(0.2);
        sim.startAuto(autoName);
        Command auto = sim.rc.getAutonomousCommand();
        System.out.printf("[SIM] auto '%s' selected: %s%n", autoName, auto == null ? "NOTHING SELECTED" : auto.getName());
        assertTrue(auto != null, "the chooser did not select " + autoName);

        List<SuperStructureState> states = new ArrayList<>();
        boolean floorRaisedWhileIntaking = false;
        Pose2d first = null;
        double finished = -1;
        long startNanos = System.nanoTime();
        for (int i = 0; i < 45 * 50; i++) { // up to 45 s
            sim.loop();
            if (first == null) first = sim.rc.drivetrain.getState().Pose;
            var st = sim.rc.superStructure.getCurrentState();
            if (states.isEmpty() || states.get(states.size() - 1) != st) states.add(st);
            if (st == SuperStructureState.INTAKE && sim.rc.fourBarHandler.getCurrentState() == FourBarState.RAISED) floorRaisedWhileIntaking = true;
            if (!CommandScheduler.getInstance().isScheduled(auto)) {
                finished = (System.nanoTime() - startNanos) / 1e9;
                break;
            }
        }
        sim.run(1.0);
        Pose2d end = sim.rc.drivetrain.getState().Pose;
        System.out.printf("[SIM] auto '%s': finished after %.1f s, started near (%.1f, %.1f) ended at (%.2f, %.2f), states %s%n",
            autoName, finished, first.getX(), first.getY(), end.getX(), end.getY(), states);
        assertTrue(finished > 0, "the auto never finished within 45 s");
        assertTrue(states.contains(SuperStructureState.INTAKE), "the auto never ran the intake");
        assertFalse(floorRaisedWhileIntaking, "the floor lifted while intaking during the auto");
        assertTrue(end.getX() < 2.0 && end.getY() < 2.0, "the auto should end at the outpost, ended at " + end);
        sim.endAuto();
    }

    @org.junit.jupiter.api.AfterEach
    void noCollisionsWhateverHappened() {
        RobotSim.assertNoViolationsNow();
    }
}
