// code by jph
package ch.ethz.idsc.owly.demo.se2r.glc;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.se2r.Se2rGoalManager;
import ch.ethz.idsc.owly.demo.util.ImageRegions;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.Se2Utils;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** (x,y,theta) */
enum Se2rImageDemo {
  ;
  public static void main(String[] args) throws Exception {
    ImageRegion imageRegion = ImageRegions.loadFromRepository("/io/track0_100.png", Tensors.vector(10, 10), false);
    Tensor partitionScale = Tensors.vector(3, 3, 50 / Math.PI);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, RationalScalar.of(1, 6), 5);
    Collection<Flow> controls = Se2Controls.createControlsForwardAndReverse(Se2Utils.DEGREE(45), 6);
    Se2rGoalManager se2GoalManager = new Se2rGoalManager( //
        Tensors.vector(5.3, 4.4), RealScalar.of(0), //
        RealScalar.of(.1), Se2Utils.DEGREE(10));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(imageRegion));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        partitionScale, stateIntegrator, controls, obstacleQuery, se2GoalManager.getGoalInterface());
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0, 0));
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(179, 448);
    owlyFrame.jFrame.setBounds(100, 100, 700, 700);
    owlyFrame.addBackground(imageRegion);
    while (!trajectoryPlanner.getBest().isPresent() && owlyFrame.jFrame.isVisible()) {
      Expand.maxSteps(trajectoryPlanner, 1000);
      owlyFrame.setGlc(trajectoryPlanner);
      Thread.sleep(10);
    }
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
      StateTimeTrajectories.print(trajectory);
    }
  }
}
