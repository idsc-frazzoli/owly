// code by jph
package ch.ethz.idsc.owly.demo.lv.demo;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.lv.LvControls;
import ch.ethz.idsc.owly.demo.lv.LvGoalInterface;
import ch.ethz.idsc.owly.demo.lv.LvStateSpaceModel;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.state.EmptyTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Log;

/** this demo deliberately does not create a tree but
 * only shows the integration of a single trajectory
 * 
 * the coordinates represent the population of predators and prey */
enum LvDemo {
  ;
  public static void main(String[] args) {
    Tensor eta = Tensors.vector(10, 10);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, RationalScalar.of(1, 30), 4);
    StateSpaceModel stateSpaceModel = LvStateSpaceModel.of(1, 2);
    Collection<Flow> controls = LvControls.create(stateSpaceModel, 2);
    GoalInterface goalInterface = new LvGoalInterface(Tensors.vector(2, 1), Tensors.vector(.1, .1));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, EmptyTrajectoryRegionQuery.INSTANCE, goalInterface);
    // ---
    trajectoryPlanner.represent = Log::of;
    trajectoryPlanner.insertRoot(Tensors.vector(2, .1));
    Expand.maxSteps(trajectoryPlanner, 4000);
    Gui.glc(trajectoryPlanner);
  }
}
