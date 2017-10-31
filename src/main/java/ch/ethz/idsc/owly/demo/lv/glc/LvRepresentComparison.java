// code by jph
package ch.ethz.idsc.owly.demo.lv.glc;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.lv.LvControls;
import ch.ethz.idsc.owly.demo.lv.LvGoalInterface;
import ch.ethz.idsc.owly.demo.lv.LvStateSpaceModel;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.OwlyFrame;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateTimeTensorFunction;
import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.state.EmptyTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Log;

/** the coordinates represent the population of predators and prey.
 * the domain coordinates are computed from the log of the state coordinates */
enum LvRepresentComparison {
  ;
  static void launch(TensorUnaryOperator represent) {
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
    trajectoryPlanner.represent = StateTimeTensorFunction.state(represent);
    trajectoryPlanner.insertRoot(Tensors.vector(2, .5));
    Expand.maxSteps(trajectoryPlanner, 4000);
    OwlyFrame owlyFrame = OwlyGui.glc(trajectoryPlanner);
    owlyFrame.configCoordinateOffset(100, 300);
    owlyFrame.jFrame.setBounds(100, 100, 500, 500);
  }

  public static void main(String[] args) {
    launch(Log::of);
    launch(t -> t);
  }
}
