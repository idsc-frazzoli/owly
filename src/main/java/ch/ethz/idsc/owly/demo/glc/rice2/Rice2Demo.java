// code by jph
package ch.ethz.idsc.owly.demo.glc.rice2;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.MidpointIntegrator;
import ch.ethz.idsc.owly.math.region.HyperplaneRegion;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

class Rice2Demo {
  // TODO in general ensure that goal region contains at least 1 domain etc.
  public static void main(String[] args) {
    Tensor eta = Tensors.vector(3, 3, 6, 6);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        new MidpointIntegrator(), RationalScalar.of(1, 2), 5);
    Collection<Flow> controls = Rice2Controls.createControls(RealScalar.of(.5), 3, 15);
    Rice2GoalManager rice2Goal = new Rice2GoalManager( //
        Tensors.vector(3, 3, -1, 0), Tensors.vector(.5, .5, .4, .4));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new HyperplaneRegion(Tensors.vector(1, +0, 0, 0), RealScalar.ZERO), //
                new HyperplaneRegion(Tensors.vector(0, +1, 0, 0), RealScalar.ZERO), //
                new HyperplaneRegion(Tensors.vector(0, -1, 0, 0), RealScalar.of(3.2)), //
                new HyperplaneRegion(Tensors.vector(0, +0, 0, 1), RealScalar.ZERO) //
            )));
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        eta, stateIntegrator, controls, rice2Goal, rice2Goal, obstacleQuery);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0.1, 0.1, 0, 0));
    long tic = System.nanoTime();
    int iters = Expand.maxSteps(trajectoryPlanner, 1000);
    long toc = System.nanoTime();
    // 550 1.6898229210000002 without parallel integration of trajectories
    // 555 1.149214356 with parallel integration of trajectories
    System.out.println(iters + " " + ((toc - tic) * 1e-9));
    // TODO keep trying to improve path to goal for a few iterations...?
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    Gui.glc(trajectoryPlanner);
  }
}
