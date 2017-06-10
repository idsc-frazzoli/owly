// code by jph
package ch.ethz.idsc.owly.demo.glc.rice1;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
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

/** "Mobility and Autonomous Reconfiguration of Marsokhod" */
class Rice1Demo {
  public static void main(String[] args) {
    Tensor eta = Tensors.vector(5, 8);
    StateIntegrator stateIntegrator = FixedStateIntegrator.createDefault(RationalScalar.of(1, 5), 5);
    Collection<Flow> controls = Rice1Controls.createControls(RealScalar.of(.5), 15); //
    Rice1GoalManager rice1Goal = new Rice1GoalManager(Tensors.vector(6, -.7), Tensors.vector(.4, .3));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new EllipsoidRegion(Tensors.vector(+3, +1), Tensors.vector(1.75, .75)),
                // , // speed limit along the way
                new EllipsoidRegion(Tensors.vector(-2, +0), Tensors.vector(1, 1)) // block to the left
            )));
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, rice1Goal);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    int iters = Expand.maxSteps(trajectoryPlanner, 1000);
    System.out.println(iters);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    Gui.glc(trajectoryPlanner);
  }
}
