// code by edo
package ch.ethz.idsc.owly.demo.drift;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.NegativeHalfspaceRegion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum DriftDemo {
  ;
  public static void main(String[] args) {
    Collection<Flow> controls = DriftControls.create(10);
    Tensor eta = Tensors.vector(30, 30, 5);
    StateIntegrator stateIntegrator = //
        FixedStateIntegrator.createDefault(RationalScalar.of(1, 10), 7);
    System.out.println("scale=" + eta);
    GoalInterface goalInterface = DriftGoalManager.createStandard(//
        Tensors.vector(-0.3055, 0.5032, 8), //
        Tensors.vector(0.05, 0.05, 0.25));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new NegativeHalfspaceRegion(1)));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, goalInterface);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0, 4));
    int iters = Expand.maxSteps(trajectoryPlanner, 5000);
    System.out.println(iters);
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<GlcNode> trajectory = Nodes.listFromRoot(optional.get());
      for (GlcNode node : trajectory) {
        if (!node.isRoot())
          System.out.println(node.flow().getU());
        System.out.println(node.stateTime().toInfoString());
      }
      // StateTimeTrajectories.print(trajectory);
    }
    OwlyGui.glc(trajectoryPlanner);
  }
}
