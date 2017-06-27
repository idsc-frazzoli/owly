// code by jph
package ch.ethz.idsc.owly.demo.lv.demo;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.lv.LvControls;
import ch.ethz.idsc.owly.demo.lv.LvGoalInterface;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.state.EmptyTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** this demo deliberately does not create a tree but
 * only shows the integration of a single trajectory
 * 
 * the coordinates represent the population of predators and prey */
class LvDemo {
  public static void main(String[] args) {
    Tensor eta = Tensors.vector(10, 10);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, RationalScalar.of(1, 20), 10);
    Collection<Flow> controls = LvControls.set();
    GoalInterface goalInterface = new LvGoalInterface();
    TrajectoryRegionQuery obstacleQuery = new EmptyTrajectoryRegionQuery();
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, goalInterface);
    // trajectoryPlanner.represent = psuWrap::represent;
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(.2, .3));
    Expand.maxSteps(trajectoryPlanner, 12);
    Gui.glc(trajectoryPlanner);
  }
}
