// code by jph
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.delta.DeltaControls;
import ch.ethz.idsc.owly.demo.delta.DeltaGoalManager;
import ch.ethz.idsc.owly.demo.delta.DeltaGoalManagerExt;
import ch.ethz.idsc.owly.demo.delta.DeltaParameters;
import ch.ethz.idsc.owly.demo.delta.DeltaStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.demo.util.Images;
import ch.ethz.idsc.owly.demo.util.Resources;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.AnyPlannerInterface;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
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

public enum DeltaHelper {
  ;
  // ---
  // don't change this function, make a separate function if necessary
  public static TrajectoryPlanner createDefault(Scalar amp) throws Exception {
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
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, deltaGoalManager);
    trajectoryPlanner.insertRoot(Tensors.vector(8.8, 0.5));
    return trajectoryPlanner;
  }

  static TrajectoryPlanner createGlc(Scalar gradientAmp, RationalScalar resolution) throws Exception {
    Scalar timeScale = RealScalar.of(5);
    Scalar depthScale = RealScalar.of(10);
    Tensor partitionScale = Tensors.vector(10e5, 10e5);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Tensor range = Tensors.vector(9, 6.5);
    ImageGradient ipr = new ImageGradient( //
        Images.displayOrientation(Import.of(Resources.fileFromRepository("/io/delta_uxy.png")).get(Tensor.ALL, Tensor.ALL, 0)), //
        range, RealScalar.of(-.5)); // -.25 .5
    DeltaStateSpaceModel stateSpaceModel = new DeltaStateSpaceModel(ipr);
    Scalar maxInput = RealScalar.ONE;
    Scalar maxSpeed = maxInput.add(ipr.maxNorm());
    Collection<Flow> controls = DeltaControls.createControls( //
        stateSpaceModel, maxInput, resolution.number().intValue());
    Parameters parameters = new DeltaParameters(resolution, timeScale, depthScale, //
        partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    System.out.println("1/DomainSize: " + parameters.getEta());
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        new RungeKutta45Integrator(), parameters.getdtMax(), parameters.getTrajectorySize());
    Tensor obstacleImage = Images.displayOrientation(Import.of(Resources.fileFromRepository("/io/delta_free.png")).get(Tensor.ALL, Tensor.ALL, 0)); //
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new ImageRegion(obstacleImage, range, true)));
    DeltaGoalManagerExt deltaGoalManager = new DeltaGoalManagerExt( //
        Tensors.vector(2.9, 2.4), Tensors.vector(.3, .3), maxSpeed);
    // DeltaGoalManager deltaGoalManager = new DeltaGoalManager( //
    // Tensors.vector(2.1, 0.3), Tensors.vector(.3, .3));
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, deltaGoalManager);
    trajectoryPlanner.insertRoot(Tensors.vector(8.8, 0.5));
    return trajectoryPlanner;
  }

  static AnyPlannerInterface createGlcAny(Scalar gradientAmp, RationalScalar resolution) throws Exception {
    Scalar timeScale = RealScalar.of(5);
    Scalar depthScale = RealScalar.of(10);
    Tensor partitionScale = Tensors.vector(10e5, 10e5);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Tensor range = Tensors.vector(9, 6.5);
    ImageGradient ipr = new ImageGradient( //
        Images.displayOrientation(Import.of(Resources.fileFromRepository("/io/delta_uxy.png")).get(Tensor.ALL, Tensor.ALL, 0)), //
        range, RealScalar.of(-.5)); // -.25 .5
    DeltaStateSpaceModel stateSpaceModel = new DeltaStateSpaceModel(ipr);
    Scalar maxInput = RealScalar.ONE;
    Scalar maxSpeed = maxInput.add(ipr.maxNorm());
    Collection<Flow> controls = DeltaControls.createControls( //
        stateSpaceModel, maxInput, resolution.number().intValue());
    Parameters parameters = new DeltaParameters(resolution, timeScale, depthScale, //
        partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    System.out.println("1/DomainSize: " + parameters.getEta());
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        new RungeKutta45Integrator(), parameters.getdtMax(), parameters.getTrajectorySize());
    Tensor obstacleImage = Images.displayOrientation(Import.of(Resources.fileFromRepository("/io/delta_free.png")).get(Tensor.ALL, Tensor.ALL, 0)); //
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            new ImageRegion(obstacleImage, range, true)));
    DeltaGoalManagerExt deltaGoalManager = new DeltaGoalManagerExt( //
        Tensors.vector(2.9, 2.4), Tensors.vector(.3, .3), maxSpeed);
    // DeltaGoalManager deltaGoalManager = new DeltaGoalManager( //
    // Tensors.vector(2.1, 0.3), Tensors.vector(.3, .3));
    AnyPlannerInterface trajectoryPlanner = new OptimalAnyTrajectoryPlanner(parameters.getEta(), stateIntegrator, controls, obstacleQuery, deltaGoalManager);
    trajectoryPlanner.switchRootToState((Tensors.vector(8.8, 0.5)));
    return trajectoryPlanner;
  }
}
