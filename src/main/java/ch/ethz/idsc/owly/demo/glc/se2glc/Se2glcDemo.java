// code by jph
package ch.ethz.idsc.owly.demo.glc.se2glc;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.glc.se2.Se2Utils;
import ch.ethz.idsc.owly.demo.glc.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.glc.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.glc.se2.Se2GoalManager;
import ch.ethz.idsc.owly.demo.glc.se2.Se2PointcloudRegion;

import ch.ethz.idsc.owly.glc.adapter.HyperplaneRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Parameters;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectories;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.RegionUnion;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** (x,y,theta) */
class Se2glcDemo {
  public static void main(String[] args) {
    Integrator integrator = new EulerIntegrator();
    Parameters parameters = new Parameters();
    
    parameters.setResolution(8);
    parameters.setDepthScale(RealScalar.of(100));
    parameters.setDtMax(RationalScalar.of(1, 5));
    parameters.setPartitionScale(parameters.)
    
    // ---
    Scalar dtMax = RationalScalar.of(1, 6);
    int trajectorySize = 5;
    int depthLimit = 1000;
    // ---
    Tensor eta = Tensors.vector(3, 3, 15); // TODO instead of 15 use multiple of PI... not 1/3?
    System.out.println("scale=" + eta);
    Collection<Flow> controls = Se2Controls.createControls(Se2Utils.DEGREE(45), 6);
    Se2GoalManager se2GoalManager = new Se2GoalManager( //
        Tensors.vector(0, 1), RealScalar.of(Math.PI), //
        DoubleScalar.of(.1), Se2Utils.DEGREE(10));
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(se2GoalManager));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new HyperplaneRegion(Tensors.vector(0, -1, 0), RealScalar.of(1.5)), //
                new HyperplaneRegion(Tensors.vector(0, +1, 0), RealScalar.of(2.0)) //
            )));
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        integrator, dtMax, eta, depthLimit, controls, trajectorySize, se2GoalManager, goalQuery, obstacleQuery);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0, 0));
    int iters = trajectoryPlanner.plan(2000);
    System.out.println(iters);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
