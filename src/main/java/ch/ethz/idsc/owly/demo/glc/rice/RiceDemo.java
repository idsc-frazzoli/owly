// code by jph
package ch.ethz.idsc.owly.demo.glc.rice;

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
import ch.ethz.idsc.owly.math.UnionRegion;
import ch.ethz.idsc.owly.math.integrator.Integrator;
import ch.ethz.idsc.owly.math.integrator.MidpointIntegrator;
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
    DynamicalSystem dynamicalSystem = new DynamicalSystem() {
      @Override
      public Scalar getMaxTimeStep() {
        return RealScalar.of(.25);
      }
    };
    Controls controls = RiceControls.createControls(15);
    CostFunction costFunction = new MinTimeCost();
    Heuristic heuristic = new ZeroHeuristic();
    TrajectoryRegionQuery goalQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new EllipsoidRegion(Tensors.vector(6, -.7), Tensors.vector(.1, .1)) //
        ));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            UnionRegion.of( //
                new EllipsoidRegion(Tensors.vector(3, +1), Tensors.vector(.75, .75)), // speed limit along the way
                new EllipsoidRegion(Tensors.vector(-2, 0), Tensors.vector(1, 1)) // block to the left
            )));
    // ---
    TrajectoryPlanner trajectoryPlanner = new TrajectoryPlanner( //
        integrator, //
        dynamicalSystem, controls, costFunction, heuristic, goalQuery, obstacleQuery);
    // ---
    trajectoryPlanner.setResolution(Tensors.vector(13, 13));
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    trajectoryPlanner.plan(25);
    Trajectory trajectory = trajectoryPlanner.getPathFromRootToGoal();
    trajectory.print();
    GlcFrame glcFrame = new GlcFrame();
    glcFrame.glcComponent.setTrajectoryPlanner(trajectoryPlanner);
  }
}
