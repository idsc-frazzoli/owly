// code by jph
package ch.ethz.idsc.owly.demo.glc.se2r;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.zip.DataFormatException;

import ch.ethz.idsc.owly.demo.glc.se2.Se2Utils;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectories;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.ImageRegion;
import ch.ethz.idsc.owly.math.Region;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.Import;

/** (x,y,theta) */
class Se2rImageDemo {
  public static void main(String[] args) throws ClassNotFoundException, DataFormatException, IOException {
    Integrator integrator = new EulerIntegrator();
    Region region;
    {
      String string = integrator.getClass().getResource("/io/track0_100.png").getPath();
      Tensor image = Import.of(new File(string)).get(Tensor.ALL, Tensor.ALL, 0);
      region = new ImageRegion(image, Tensors.vector(10, 10));
    }
    Scalar timeStep = RationalScalar.of(1, 6);
    Tensor partitionScale = Tensors.vector(3, 3, 15); // TODO instead of 15 use multiple of PI...
    Collection<Flow> controls = Se2rControls.createControls(Se2Utils.DEGREE(45), 6);
    int trajectorySize = 5;
    Se2rGoalManager se2GoalManager = new Se2rGoalManager( //
        Tensors.vector(5.3, 4.4), RealScalar.of(0), //
        RealScalar.of(.1), Se2Utils.DEGREE(10));
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(se2GoalManager));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(region));
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        integrator, timeStep, partitionScale, controls, trajectorySize, //
        se2GoalManager, goalQuery, obstacleQuery);
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0, 0));
    int iters = trajectoryPlanner.plan(10000);
    System.out.println(iters);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
