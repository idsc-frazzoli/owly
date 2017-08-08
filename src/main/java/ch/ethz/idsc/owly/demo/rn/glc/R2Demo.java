// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.rn.R2Bubbles;
import ch.ethz.idsc.owly.demo.rn.R2Controls;
import ch.ethz.idsc.owly.demo.rn.RnSimpleCircleHeuristicGoalManager;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.EmptyTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

enum R2Demo {
  ;
  public static TrajectoryPlanner simpleEmpty() {
    return simple(EmptyTrajectoryRegionQuery.INSTANCE);
  }

  public static TrajectoryPlanner simpleR2Bubbles() {
    return simple( //
        new SimpleTrajectoryRegionQuery( //
            new TimeInvariantRegion(new R2Bubbles())));
  }

  private static TrajectoryPlanner simple(TrajectoryRegionQuery obstacleQuery) {
    final Tensor stateRoot = Tensors.vector(-2, -2);
    final Tensor stateGoal = Tensors.vector(2, 2);
    final Scalar radius = DoubleScalar.of(.25);
    // ---
    Tensor eta = Tensors.vector(4, 4);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RationalScalar.of(1, 5), 5);
    Collection<Flow> controls = R2Controls.createRadial(36);
    RnSimpleCircleHeuristicGoalManager rnGoal = new RnSimpleCircleHeuristicGoalManager(stateGoal, radius);
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, rnGoal);
    trajectoryPlanner.insertRoot(stateRoot);
    int iters = Expand.maxSteps(trajectoryPlanner, 200);
    System.out.println("iterations " + iters);
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      GlcNode goalNode = optional.get(); // <- throws exception if
      Scalar cost = goalNode.costFromRoot();
      Scalar lowerBound = Ramp.of(Norm._2.of(stateGoal.subtract(stateRoot)).subtract(radius));
      if (Scalars.lessThan(cost, lowerBound))
        throw TensorRuntimeException.of(cost, lowerBound);
    }
    return trajectoryPlanner;
  }

  public static void main(String[] args) {
    {
      TrajectoryPlanner trajectoryPlanner = simpleEmpty();
      Optional<GlcNode> optional = trajectoryPlanner.getBest();
      if (optional.isPresent()) {
        List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
        StateTimeTrajectories.print(trajectory);
      }
      Gui.glc(trajectoryPlanner);
    }
    {
      TrajectoryPlanner trajectoryPlanner = simpleR2Bubbles();
      Optional<GlcNode> optional = trajectoryPlanner.getBest();
      if (optional.isPresent()) {
        List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
        StateTimeTrajectories.print(trajectory);
      }
      Gui.glc(trajectoryPlanner);
    }
  }
}
