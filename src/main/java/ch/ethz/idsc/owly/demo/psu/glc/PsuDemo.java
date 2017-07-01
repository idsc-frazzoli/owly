// code by jph
package ch.ethz.idsc.owly.demo.psu.glc;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.psu.PsuControls;
import ch.ethz.idsc.owly.demo.psu.PsuGoalManager;
import ch.ethz.idsc.owly.demo.psu.PsuWrap;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.flow.RungeKutta4Integrator;
import ch.ethz.idsc.owly.math.state.EmptyTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** Pendulum Swing-up
 * 
 * bapaden phd thesis: 5.5.2 Torque-Limited Pendulum Swing-Up
 * 
 * "A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning" [Paden/Frazzoli] */
public enum PsuDemo { // <- intentionally public
  ;
  public static TrajectoryPlanner simple() {
    Tensor eta = Tensors.vector(5, 7);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta4Integrator.INSTANCE, RationalScalar.of(1, 4), 5);
    Collection<Flow> controls = PsuControls.createControls(0.2, 6);
    PsuWrap psuWrap = new PsuWrap();
    PsuGoalManager psuGoalManager = new PsuGoalManager(psuWrap, //
        Tensors.vector(Math.PI * 0.7, .5), RealScalar.of(0.3));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, EmptyTrajectoryRegionQuery.INSTANCE, psuGoalManager.getGoalInterface());
    trajectoryPlanner.represent = psuWrap::represent;
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    Expand.maxSteps(trajectoryPlanner, 1000);
    return trajectoryPlanner;
  }

  public static TrajectoryPlanner medium() {
    Tensor eta = Tensors.vector(5, 7);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, RationalScalar.of(1, 4), 5);
    Collection<Flow> controls = PsuControls.createControls(0.2, 6);
    PsuWrap psuWrap = new PsuWrap();
    PsuGoalManager psuGoalManager = new PsuGoalManager(psuWrap, //
        Tensors.vector(Math.PI, 2), RealScalar.of(0.3));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, EmptyTrajectoryRegionQuery.INSTANCE, psuGoalManager.getGoalInterface());
    trajectoryPlanner.represent = psuWrap::represent;
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    int iters = Expand.maxSteps(trajectoryPlanner, 1000);
    System.out.println(iters);
    return trajectoryPlanner;
  }

  public static void main(String[] args) {
    TrajectoryPlanner trajectoryPlanner = medium();
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
      Trajectories.print(trajectory);
    }
    Gui.glc(trajectoryPlanner);
  }
}
