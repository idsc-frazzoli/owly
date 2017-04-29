// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.zip.DataFormatException;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectories;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.PolygonRegion;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.io.Import;

class R2ImageDemo {
  public static void main(String[] args) {
    Integrator integrator = new EulerIntegrator();
    String string = integrator.getClass().getResource("/io/track0_100.png").getPath();
    try {
      Tensor image = Import.of(new File(string));
      System.out.println(Dimensions.of(image));
    } catch (ClassNotFoundException | DataFormatException | IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    // File file = ;
    // System.out.println(file.exists());
    // System.exit(1);
    final Scalar timeStep = RationalScalar.of(1, 8);
    Tensor partitionScale = Tensors.vector(5, 5);
    Collection<Flow> controls = R2Controls.createControls(20);
    int trajectorySize = 4;
    RnGoalManager rnGoal = new RnGoalManager(Tensors.vector(5, 5), DoubleScalar.of(.2));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new PolygonRegion(Tensors.matrix(new Number[][] { //
                { 3, 1 }, //
                { 4, 1 }, //
                { 4, 4 }, //
                { 1, 4 }, //
                { 1, 3 }, //
                { 3, 3 }//
            }))));
    // ---
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        integrator, timeStep, partitionScale, controls, trajectorySize, //
        rnGoal, rnGoal, obstacleQuery);
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    int iters = trajectoryPlanner.plan(1000);
    System.out.println(iters);
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
