// code by jph
package ch.ethz.idsc.owly.demo.glc.tn;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.util.R2Controls;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.EmptyTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

class T2Demo {
  public static void main(String[] args) {
    Tensor eta = Tensors.vector(4, 4);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(new EulerIntegrator(), RationalScalar.of(1, 10), 5);
    Collection<Flow> controls = R2Controls.createRadial(36);
    TnGoalManager rnGoal = new TnGoalManager(Tensors.vector(2, 2), RealScalar.of(.25), null); // TODO
    // performance depends on heuristic: zeroHeuristic vs rnGoal
    // Heuristic heuristic = new ZeroHeuristic(); // rnGoal
    TrajectoryRegionQuery obstacleQuery = new EmptyTrajectoryRegionQuery();
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        eta, stateIntegrator, controls, rnGoal, rnGoal, obstacleQuery);
    trajectoryPlanner.insertRoot(Tensors.vector(-2, -2));
    Expand.maxSteps(trajectoryPlanner, 1400);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    Gui.glc(trajectoryPlanner);
  }
}
