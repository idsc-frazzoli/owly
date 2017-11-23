// code by jph
package ch.ethz.idsc.owly.demo.psu.glc;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.gui.ani.OwlyGui;
import ch.ethz.idsc.owl.math.StateTimeTensorFunction;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owl.math.flow.RungeKutta4Integrator;
import ch.ethz.idsc.owl.math.state.EmptyTrajectoryRegionQuery;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owly.demo.psu.PsuControls;
import ch.ethz.idsc.owly.demo.psu.PsuGoalManager;
import ch.ethz.idsc.owly.demo.psu.PsuWrap;
import ch.ethz.idsc.owly.glc.adapter.Expand;
import ch.ethz.idsc.owly.glc.adapter.GlcNodes;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.std.StandardTrajectoryPlanner;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;

/** Pendulum Swing-up
 * 
 * bapaden phd thesis: 5.5.2 Torque-Limited Pendulum Swing-Up
 * 
 * "A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning" [Paden/Frazzoli] */
enum PsuDemo {
  ;
  public static TrajectoryPlanner simple() {
    Tensor eta = Tensors.vector(5, 7);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta4Integrator.INSTANCE, RationalScalar.of(1, 4), 5);
    Collection<Flow> controls = PsuControls.createControls(0.2, 6);
    PsuWrap psuWrap = PsuWrap.INSTANCE;
    GoalInterface goalInterface = PsuGoalManager.of( //
        psuWrap, Tensors.vector(Math.PI * 0.7, .5), RealScalar.of(0.3));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, EmptyTrajectoryRegionQuery.INSTANCE, goalInterface);
    trajectoryPlanner.represent = StateTimeTensorFunction.state(psuWrap::represent);
    // ---
    trajectoryPlanner.insertRoot(new StateTime(Array.zeros(2), RealScalar.ZERO));
    Expand.maxSteps(trajectoryPlanner, 1000);
    return trajectoryPlanner;
  }

  public static TrajectoryPlanner medium() {
    Tensor eta = Tensors.vector(5, 7);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, RationalScalar.of(1, 4), 5);
    Collection<Flow> controls = PsuControls.createControls(0.2, 6);
    PsuWrap psuWrap = PsuWrap.INSTANCE;
    GoalInterface goalInterface = PsuGoalManager.of( //
        psuWrap, Tensors.vector(Math.PI, 2), RealScalar.of(0.3));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, EmptyTrajectoryRegionQuery.INSTANCE, goalInterface);
    trajectoryPlanner.represent = StateTimeTensorFunction.state(psuWrap::represent);
    // ---
    trajectoryPlanner.insertRoot(new StateTime(Array.zeros(2), RealScalar.ZERO));
    int iters = Expand.maxSteps(trajectoryPlanner, 1000);
    System.out.println(iters);
    return trajectoryPlanner;
  }

  public static void main(String[] args) {
    TrajectoryPlanner trajectoryPlanner = medium();
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
      StateTimeTrajectories.print(trajectory);
    }
    OwlyGui.glc(trajectoryPlanner);
  }
}
