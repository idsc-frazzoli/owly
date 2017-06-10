// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.util.R2Controls;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.PolygonRegion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

class R2PolygonDemo {
  public static void main(String[] args) {
    Tensor partitionScale = Tensors.vector(5, 5);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(new EulerIntegrator(), RationalScalar.of(1, 8), 4);
    Collection<Flow> controls = R2Controls.createRadial(20);
    RnGoalManager rnGoal = new RnGoalManager(Tensors.vector(5, 5), DoubleScalar.of(.2));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new PolygonRegion(Tensors.matrix(new Number[][] { //
                { 3, 0 }, //
                { 4, 0 }, //
                { 4, 6 }, //
                { 1, 6 }, //
                { 1, 3 }, //
                { 3, 3 }//
            }))));
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        partitionScale, stateIntegrator, controls, obstacleQuery, rnGoal);
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    int iters = Expand.maxSteps(trajectoryPlanner, 1500);
    System.out.println(iters);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    Gui.glc(trajectoryPlanner);
  }
}
