// code by jph
package ch.ethz.idsc.owly.demo.glc.delta;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.util.Images;
import ch.ethz.idsc.owly.demo.util.Resources;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.Import;

class DeltaDemo {
  public static void main(String[] args) throws Exception {
    Tensor eta = Tensors.vector(9, 9);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        new RungeKutta45Integrator(), RationalScalar.of(1, 10), 4);
    Tensor range = Tensors.vector(9, 6.5);
    ImageGradient ipr = new ImageGradient( //
        Images.displayOrientation(Import.of(Resources.fileFromRepository("/io/delta_uxy.png")).get(Tensor.ALL, Tensor.ALL, 0)), //
        range, RealScalar.of(-.5)); // -.25 .5
    RealScalar maxInput = RealScalar.of(1);
    Collection<Flow> controls = DeltaControls.createControls( //
        new DeltaStateSpaceModel(ipr), maxInput, 35);
    Tensor obstacleImage = Images.displayOrientation(Import.of(Resources.fileFromRepository("/io/delta_free.png")).get(Tensor.ALL, Tensor.ALL, 0)); //
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new ImageRegion(obstacleImage, range, true)));
    DeltaGoalManager deltaGoalManager = new DeltaGoalManager( //
        Tensors.vector(2.1, 0.3), Tensors.vector(.3, .3), maxInput);
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        eta, stateIntegrator, controls, deltaGoalManager, deltaGoalManager, obstacleQuery);
    trajectoryPlanner.insertRoot(Tensors.vector(8.8, 0.5));
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    while (trajectoryPlanner.getBest() == null && owlyFrame.jFrame.isVisible()) {
      Expand.maxSteps(trajectoryPlanner, 10);
      owlyFrame.setGlc(trajectoryPlanner);
      Thread.sleep(1);
    }
  }
}
