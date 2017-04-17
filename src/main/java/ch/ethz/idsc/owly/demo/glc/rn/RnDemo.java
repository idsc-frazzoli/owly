// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.owly.glc.adapter.MinTimeCost;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.adapter.ZeroHeuristic;
import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.DynamicalSystem;
import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.gui.GlcFrame;
import ch.ethz.idsc.owly.math.EllipsoidRegion;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.integrator.EulerIntegrator;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.owly.uni.adapter.IdentityStateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;

public class RnDemo {
  public static void main(String[] args) {
    Integrator integrator = new EulerIntegrator();
    StateSpaceModel stateSpaceModel = new IdentityStateSpaceModel();
    DynamicalSystem dynamicalSystem = new RnSingleIntegrator(RealScalar.ONE);
    Controls controls = RnControls.createR2RadialControls(40, RealScalar.of(.7));
    // System.out.println(Pretty.of(controls));
    CostFunction costFunction = new MinTimeCost();
    Heuristic heuristic = new ZeroHeuristic();
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new EllipsoidRegion(Tensors.vector(10, 0), Tensors.vector(.5, .5))));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new EllipsoidRegion(Tensors.vector(5, 0), Tensors.vector(4, 4))));
    // ---
    TrajectoryPlanner trajectoryPlanner = new TrajectoryPlanner( //
        integrator, //
        stateSpaceModel, //
        dynamicalSystem, controls, costFunction, heuristic, goalQuery, obstacleQuery);
    // ---
    trajectoryPlanner.initialize(Tensors.vector(3, 3));
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    trajectoryPlanner.plan();
    Trajectory trajectory = trajectoryPlanner.getPathFromRootToGoal();
    trajectory.print();
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
    // double d = Double.parseDouble("0.5000000000000001");
    // double dn = Math.nextDown(d);
    // System.out.println(dn);
  }
}
