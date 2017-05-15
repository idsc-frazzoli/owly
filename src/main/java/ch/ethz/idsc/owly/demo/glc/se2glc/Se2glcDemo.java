// code by jph
package ch.ethz.idsc.owly.demo.glc.se2glc;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.glc.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.glc.se2.Se2GoalManager;
import ch.ethz.idsc.owly.demo.glc.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.glc.se2.Se2Utils;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.wrap.Parameters;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.HyperplaneRegion;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** (x,y,theta) */
class Se2glcDemo {
  public static void main(String[] args) throws Exception {
    int resolution = 8;
    Scalar timeScale = RealScalar.of(10);
    Scalar depthScale = RealScalar.of(5);
    Tensor partitionScale = Tensors.vector(3, 3, 15);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    StateSpaceModel stateSpaceModel = new Se2StateSpaceModel();
    // --
    Parameters parameters = new Se2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    // TODO possible without creation of StateSpaceModel?
    StateIntegrator stateIntegrator = FixedStateIntegrator.createDefault(parameters.getdtMax(), //
        parameters.getTrajectorySize());
    // ---
    System.out.println("scale=" + parameters.getEta());
    parameters.printResolution();
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
        parameters.getEta(), stateIntegrator, controls, se2GoalManager, goalQuery, obstacleQuery);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0, 0));
    int iters = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    System.out.println("After " + iters + "iterations");
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    // swtch root
    // tree2 = ...
    Gui.glc(trajectoryPlanner);
    // glcFrame.glcComponent.addTrajectoryPlanner(trajectoryPlanner);
  }
}
