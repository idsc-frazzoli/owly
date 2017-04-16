// code by jph
package ch.ethz.idsc.owly.demo.glc.rice;

import ch.ethz.idsc.owly.adapter.RiceStateSpaceModel;
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
import ch.ethz.idsc.owly.integrator.Integrator;
import ch.ethz.idsc.owly.integrator.MidpointIntegrator;
import ch.ethz.idsc.owly.util.StateSpaceModel;
import ch.ethz.idsc.owly.util.UnionRegion;
import ch.ethz.idsc.owly.util.rn.RnSphericalRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;

/** Pendulum Swing-up
 * 
 * implementation inspired by
 * "A Generalized Label Correcting Method for Optimal Kinodynamic Motion Planning" [Paden/Frazzoli] */
public class RiceDemo {
  public static void main(String[] args) {
    Integrator integrator = new MidpointIntegrator();
    StateSpaceModel stateSpaceModel = new RiceStateSpaceModel(RealScalar.of(.5));
    DynamicalSystem dynamicalSystem = new DynamicalSystem() {
      @Override
      public Scalar getMaxTimeStep() {
        return RealScalar.of(.25);
      }
    };
    Controls controls = RiceControls.createControls(8);
    // System.out.println(Pretty.of(controls));
    CostFunction costFunction = new MinTimeCost();
    Heuristic heuristic = new ZeroHeuristic();
    // TODO join +PI and -PI
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new RnSphericalRegion(Tensors.vector(6, -.7), RealScalar.of(.1)) //
        ));
    TrajectoryRegionQuery obstacleQuery = // new EmptyRegionQuery();
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            UnionRegion.of( //
                new RnSphericalRegion(Tensors.vector(3, 1), RealScalar.of(.75)), //
                new RnSphericalRegion(Tensors.vector(3, -1), RealScalar.of(.75)) //
            )));
    // ---
    TrajectoryPlanner trajectoryPlanner = new TrajectoryPlanner( //
        integrator, stateSpaceModel, //
        dynamicalSystem, controls, costFunction, heuristic, goalQuery, obstacleQuery);
    // ---
    trajectoryPlanner.initialize(Tensors.vector(13, 13));
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    trajectoryPlanner.plan();
    Trajectory trajectory = trajectoryPlanner.getPathFromRootToGoal();
    trajectory.print();
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
