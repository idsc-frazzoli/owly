// code by jph
package ch.ethz.idsc.owly.demo.glc.se2r;

import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.zip.DataFormatException;

import ch.ethz.idsc.owly.demo.glc.se2.Se2Utils;
import ch.ethz.idsc.owly.demo.util.ImageRegions;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
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

/** (x,y,theta) */
class Se2rImageDemo {
  public static void main(String[] args) throws ClassNotFoundException, DataFormatException, IOException {
    Region region = ImageRegions.load("/io/track0_100.png", Tensors.vector(10, 10));
    Tensor partitionScale = Tensors.vector(3, 3, 15); // TODO instead of 15 use multiple of PI...
    StateIntegrator stateIntegrator = FixedStateIntegrator.createDefault(RationalScalar.of(1, 6), 5);
    Collection<Flow> controls = Se2rControls.createControls(Se2Utils.DEGREE(45), 6);
    Se2rGoalManager se2GoalManager = new Se2rGoalManager( //
        Tensors.vector(5.3, 4.4), RealScalar.of(0), //
        RealScalar.of(.1), Se2Utils.DEGREE(10));
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(se2GoalManager));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(region));
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        partitionScale, stateIntegrator, controls, se2GoalManager, goalQuery, obstacleQuery);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0, 0));
    int iters = Expand.maxSteps(trajectoryPlanner, 10000);
    System.out.println(iters);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
