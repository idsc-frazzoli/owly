// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.se2.Se2Wrap;
import ch.ethz.idsc.owly.demo.se2.Se2WrapGoalManager;
import ch.ethz.idsc.owly.glc.adapter.IdentityWrap;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.CoordinateWrap;
import ch.ethz.idsc.owly.math.RotationUtils;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.PolygonRegion;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** Notice:
 * 
 * CLASS Se2WrapDemo IS USED IN TESTS
 * MODIFICATION MAY INVALIDATE THE TESTS
 * 
 * (x,y,theta) */
enum Se2WrapDemo {
  ;
  static TrajectoryRegionQuery obstacleQuery() {
    return new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(RegionUnion.of( //
        new PolygonRegion(Tensors.matrixDouble(new double[][] { //
            { 0.633, -0.333 }, { 1.733, 0.517 }, { 1.617, 2.317 }, { 0.483, 3.317 }, //
            { -1.250, 3.167 }, { -1.383, 4.483 }, { 6.350, 4.400 }, { 6.250, -0.950 } //
        })), //
        new PolygonRegion(Tensors.matrixDouble(new double[][] { //
            { -0.717, 3.583 }, { -2.100, 1.517 }, { -3.167, 0.033 }, { -5.750, 0.017 }, { -5.517, 5.117 } //
        })), //
        new PolygonRegion(Tensors.matrixDouble(new double[][] { //
            { -6.933, 0.300 }, { -4.700, 0.250 }, { -4.617, -2.950 }, { 0.433, -3.217 }, //
            { 1.050, -0.300 }, { 1.867, -0.417 }, { 2.150, -5.300 }, { -6.900, -4.900 } //
        })) //
    )));
  }

  static TrajectoryPlanner createPlanner(CoordinateWrap coordinateWrap) {
    Tensor eta = Tensors.vector(3, 3, 50 / Math.PI);
    StateIntegrator stateIntegrator = FixedStateIntegrator.createDefault(RationalScalar.of(1, 6), 5);
    Collection<Flow> controls = Se2Controls.createControls(RotationUtils.DEGREE(45), 6);
    Tensor GOAL = Tensors.vector(-.5, 0, 0);
    Se2WrapGoalManager se2GoalManager = new Se2WrapGoalManager(coordinateWrap, GOAL, RealScalar.of(.25));
    TrajectoryRegionQuery obstacleQuery = obstacleQuery();
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, se2GoalManager.getGoalInterface());
    trajectoryPlanner.represent = coordinateWrap::represent;
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(.1, 0, 0));
    return trajectoryPlanner;
  }

  private static void demo(CoordinateWrap coordinateWrap) {
    TrajectoryPlanner trajectoryPlanner = createPlanner(coordinateWrap);
    int iters = Expand.maxSteps(trajectoryPlanner, 4000);
    System.out.println(iters);
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
      StateTimeTrajectories.print(trajectory);
    }
    Gui.glc(trajectoryPlanner);
  }

  public static void main(String[] args) {
    demo(new Se2Wrap(Tensors.vector(1, 1, 1)));
    demo(IdentityWrap.INSTANCE);
  }
}
