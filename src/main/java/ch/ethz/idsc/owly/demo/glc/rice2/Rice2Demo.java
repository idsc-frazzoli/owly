// code by jph
package ch.ethz.idsc.owly.demo.glc.rice2;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.HyperplaneRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectories;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.RegionUnion;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.owly.math.flow.MidpointIntegrator;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** "Mobility and Autonomous Reconfiguration of Marsokhod" */
class Rice2Demo {
  // TODO in general ensure that goal region contains at least 1 domain etc.
  public static void main(String[] args) {
    Integrator integrator = new MidpointIntegrator();
    Scalar timeStep = RationalScalar.of(1, 2);
    Tensor partitionScale = Tensors.vector(3, 3, 6, 6);
    Collection<Flow> controls = Rice2Controls.createControls(RealScalar.of(.5), 3, 15);
    int trajectorySize = 5;
    Rice2GoalManager rice2Goal = new Rice2GoalManager( //
        Tensors.vector(3, 3, -1, 0), Tensors.vector(.5, .5, .4, .4));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new HyperplaneRegion(Tensors.vector(1, 0, 0, 0), RealScalar.of(0.1)), //
                new HyperplaneRegion(Tensors.vector(0, 1, 0, 0), RealScalar.of(0.1)), //
                new HyperplaneRegion(Tensors.vector(0, 0, 0, 1), RealScalar.of(0.1)) //
            )));
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        integrator, timeStep, partitionScale, controls, trajectorySize, //
        rice2Goal, rice2Goal, obstacleQuery);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0, 0, 0));
    int iters = trajectoryPlanner.plan(1000);
    System.out.println(iters);
    // TODO keep trying to improve path to goal for a few iterations...?
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
