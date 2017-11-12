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
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateTimeTensorFunction;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.state.EmptyTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Log;

/** the coordinates represent the population of predators and prey.
 * the domain coordinates are computed from the log of the state coordinates */
enum LvDemo {
  ;
  public static void main(String[] args) {
    Tensor eta = Tensors.vector(10, 10);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, RationalScalar.of(1, 30), 4);
    StateSpaceModel stateSpaceModel = LvStateSpaceModel.of(1, 2);
    Collection<Flow> controls = LvControls.create(stateSpaceModel, 2);
    EllipsoidRegion ellipsoidRegion = new EllipsoidRegion(Tensors.vector(2, 1), Tensors.vector(.1, .1));
    GoalInterface goalInterface = new LvGoalInterface(ellipsoidRegion);
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, EmptyTrajectoryRegionQuery.INSTANCE, goalInterface);
    // ---
    trajectoryPlanner.represent = StateTimeTensorFunction.state(Log::of);
    trajectoryPlanner.insertRoot(new StateTime(Tensors.vector(2, .1), RealScalar.ZERO));
    Expand.maxSteps(trajectoryPlanner, 4000);
    OwlyFrame owlyFrame = OwlyGui.glc(trajectoryPlanner);
    owlyFrame.addBackground(RegionRenders.create(ellipsoidRegion));
  }
}
