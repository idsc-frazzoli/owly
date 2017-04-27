// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.EllipsoidRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectories;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.owly.math.RegionUnion;
import ch.ethz.idsc.owly.math.integrator.EulerIntegrator;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

class R2SphereDemo {
  public static void main(String[] args) {
    Integrator integrator = new EulerIntegrator();
    final Scalar timeStep = RationalScalar.of(1, 8);
    Tensor partitionScale = Tensors.vector(3.5, 4);
    Collection<Flow> controls = R2Controls.createControls(20);
    int trajectorySize = 5;
    RnGoalManager rnGoal = new RnGoalManager(Tensors.vector(5, 0), DoubleScalar.of(.5));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new EllipsoidRegion(Tensors.vector(3, 3), Tensors.vector(2, 2)), //
                new EllipsoidRegion(Tensors.vector(2.5, 0), Tensors.vector(2, 1.5)) //
            )));
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        integrator, timeStep, partitionScale, controls, trajectorySize, rnGoal, rnGoal, rnGoal, obstacleQuery);
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    int iters = trajectoryPlanner.plan(1000);
    System.out.println(iters);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
