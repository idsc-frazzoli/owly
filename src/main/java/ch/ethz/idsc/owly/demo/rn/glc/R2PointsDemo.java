// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.rn.RnPointclouds;
import ch.ethz.idsc.owly.demo.rn.RnSimpleEllipsoidGoalManager;
import ch.ethz.idsc.owly.demo.util.R2Controls;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum R2PointsDemo {
  ;
  public static void main(String[] args) {
    Tensor partitionScale = Tensors.vector(5, 5);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RationalScalar.of(1, 8), 4);
    Collection<Flow> controls = R2Controls.createRadial(20);
    RnSimpleEllipsoidGoalManager rnGoal = new RnSimpleEllipsoidGoalManager(Tensors.vector(5, 5), DoubleScalar.of(.2));
    // Tensors.matrix(new Number[][] { //
    // { 2.5, 1 }, { 2.5, 0 }, { 1.5, -1.5 }, { 0, 2 }, { 3.5, -0.5 } //
    // });
    TrajectoryRegionQuery obstacleQuery = // new EmptyRegionQuery();
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RnPointclouds.createRandomRegion(10, Tensors.vector(4, 4), Tensors.vector(2, 2), RealScalar.of(0.6))));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        partitionScale, stateIntegrator, controls, obstacleQuery, rnGoal);
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    int iters = Expand.maxSteps(trajectoryPlanner, 1000);
    System.out.println(iters);
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
      Trajectories.print(trajectory);
    }
    Gui.glc(trajectoryPlanner);
  }
}
