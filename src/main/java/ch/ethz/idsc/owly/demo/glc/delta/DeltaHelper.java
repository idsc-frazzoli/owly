// code by jph
package ch.ethz.idsc.owly.demo.glc.delta;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.util.Images;
import ch.ethz.idsc.owly.demo.util.Resources;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.DefaultTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.Import;

enum DeltaHelper {
  ;
  // ---
  // don't change this function, make a separate function if necessary
  static TrajectoryPlanner createDefault(Scalar amp) throws Exception {
    Tensor eta = Tensors.vector(8, 8);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        new RungeKutta45Integrator(), RationalScalar.of(1, 10), 4);
    Tensor range = Tensors.vector(9, 6.5);
    ImageGradient ipr = new ImageGradient( //
        Images.displayOrientation(Import.of(Resources.fileFromRepository("/io/delta_uxy.png")).get(Tensor.ALL, Tensor.ALL, 0)), //
        range, amp);
    Collection<Flow> controls = DeltaControls.createControls( //
        new DeltaStateSpaceModel(ipr), RealScalar.ONE, 25);
    Tensor obstacleImage = Images.displayOrientation(Import.of(Resources.fileFromRepository("/io/delta_free.png")).get(Tensor.ALL, Tensor.ALL, 0)); //
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new ImageRegion(obstacleImage, range, true)));
    DeltaGoalManager deltaGoalManager = new DeltaGoalManager( //
        Tensors.vector(2.1, 0.3), Tensors.vector(.3, .3));
    TrajectoryPlanner trajectoryPlanner = new DefaultTrajectoryPlanner( //
        eta, stateIntegrator, controls, deltaGoalManager, deltaGoalManager, obstacleQuery);
    trajectoryPlanner.insertRoot(Tensors.vector(8.8, 0.5));
    return trajectoryPlanner;
  }
}
