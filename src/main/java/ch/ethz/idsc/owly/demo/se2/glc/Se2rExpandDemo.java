// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.util.Arrays;
import java.util.Collection;

import ch.ethz.idsc.owl.glc.adapter.Expand;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.glc.std.StandardTrajectoryPlanner;
import ch.ethz.idsc.owl.gui.ani.OwlyFrame;
import ch.ethz.idsc.owl.gui.ani.OwlyGui;
import ch.ethz.idsc.owl.math.Degree;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.HyperplaneRegion;
import ch.ethz.idsc.owl.math.region.RegionUnion;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.se2.CarFlows;
import ch.ethz.idsc.owly.demo.se2.CarStandardFlows;
import ch.ethz.idsc.owly.demo.se2.Se2CarIntegrator;
import ch.ethz.idsc.owly.demo.se2.Se2MinTimeGoalManager;
import ch.ethz.idsc.owly.demo.util.UserHome;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.io.AnimationWriter;

/** (x,y,theta) */
enum Se2rExpandDemo {
  ;
  public static void main(String[] args) throws Exception {
    Tensor eta = Tensors.vector(6, 6, 50 / Math.PI);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        Se2CarIntegrator.INSTANCE, RationalScalar.of(1, 6), 5);
    CarFlows carFlows = new CarStandardFlows(RealScalar.ONE, Degree.of(45));
    Collection<Flow> controls = carFlows.getFlows(6);
    // place holder for parameter class
    GoalInterface goalInterface = Se2MinTimeGoalManager.create( //
        Tensors.vector(-1, -1, Math.PI * 2), //
        Tensors.vector(0.1, 0.1, 0.17), controls);
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant( //
        RegionUnion.wrap(Arrays.asList( //
            new HyperplaneRegion(Tensors.vector(0, -1, 0), RealScalar.of(1.5)), //
            new HyperplaneRegion(Tensors.vector(0, +1, 0), RealScalar.of(2.0)) //
        )));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, goalInterface);
    // ---
    trajectoryPlanner.insertRoot(new StateTime(Array.zeros(3), RealScalar.ZERO));
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.configCoordinateOffset(169, 71);
    owlyFrame.jFrame.setBounds(100, 100, 300, 200);
    try (AnimationWriter gsw = AnimationWriter.of(UserHome.Pictures("se2r.gif"), 250)) {
      while (!trajectoryPlanner.getBest().isPresent() && owlyFrame.jFrame.isVisible()) {
        Expand.maxSteps(trajectoryPlanner, 1);
        owlyFrame.setGlc(trajectoryPlanner);
        gsw.append(owlyFrame.offscreen());
        Thread.sleep(10);
      }
      int repeatLast = 6;
      while (0 < repeatLast--)
        gsw.append(owlyFrame.offscreen());
    }
    System.out.println("created gif");
  }
}
