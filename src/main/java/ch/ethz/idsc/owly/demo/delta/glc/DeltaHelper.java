// code by jph and jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.delta.DeltaControls;
import ch.ethz.idsc.owly.demo.delta.DeltaHeuristicGoalManager;
import ch.ethz.idsc.owly.demo.delta.DeltaMinTimeGoalManager;
import ch.ethz.idsc.owly.demo.delta.DeltaNoHeuristicGoalManager;
import ch.ethz.idsc.owly.demo.delta.DeltaParameters;
import ch.ethz.idsc.owly.demo.delta.DeltaStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryPlannerContainer;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.ResourceData;

public enum DeltaHelper {
  ;
  // ---
  // don't change this function, make a separate function if necessary
  public static TrajectoryPlanner createDefault(Scalar amp) throws Exception {
    Tensor eta = Tensors.vector(8, 8);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, RationalScalar.of(1, 10), 4);
    Tensor range = Tensors.vector(9, 6.5);
    ImageGradient ipr = new ImageGradient(ResourceData.of("/io/delta_uxy.png"), range, amp);
    Scalar maxInput = RealScalar.ONE;
    Collection<Flow> controls = DeltaControls.createControls( //
        new DeltaStateSpaceModel(ipr, maxInput), maxInput, 25);
    Tensor obstacleImage = ResourceData.of("/io/delta_free.png"); //
    TrajectoryRegionQuery obstacleQuery = //
        SimpleTrajectoryRegionQuery.timeInvariant(new ImageRegion(obstacleImage, range, true));
    GoalInterface goalInterface = DeltaNoHeuristicGoalManager.create( //
        Tensors.vector(2.1, 0.3), RealScalar.of(.3));
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, goalInterface);
    trajectoryPlanner.insertRoot(Tensors.vector(8.8, 0.5));
    return trajectoryPlanner;
  }

  // don't change this function, make a separate function if necessary
  public static TrajectoryPlanner createMinTimeDefault(Scalar amp) throws Exception {
    Tensor eta = Tensors.vector(8, 8);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, RationalScalar.of(1, 10), 4);
    Tensor range = Tensors.vector(9, 6.5);
    ImageGradient ipr = new ImageGradient(ResourceData.of("/io/delta_uxy.png"), range, amp);
    Scalar maxInput = RealScalar.ONE;
    Collection<Flow> controls = DeltaControls.createControls( //
        new DeltaStateSpaceModel(ipr, maxInput), maxInput, 25);
    Tensor obstacleImage = ResourceData.of("/io/delta_free.png"); //
    TrajectoryRegionQuery obstacleQuery = //
        SimpleTrajectoryRegionQuery.timeInvariant(new ImageRegion(obstacleImage, range, true));
    DeltaMinTimeGoalManager deltaGoalManager = new DeltaMinTimeGoalManager( //
        Tensors.vector(2.1, 0.3), RealScalar.of(.3), controls, ipr.maxNormGradient());
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, deltaGoalManager);
    trajectoryPlanner.insertRoot(Tensors.vector(8.8, 0.5));
    return trajectoryPlanner;
  }

  static TrajectoryPlannerContainer createGlc(Scalar gradientAmp, RationalScalar resolution, Tensor partitionScale) throws Exception {
    Tensor goal = Tensors.vector(2.9, 2.4);
    return createGlcToGoal(gradientAmp, resolution, partitionScale, goal);
  }

  static TrajectoryPlannerContainer createGlcToGoal(Scalar gradientAmp, RationalScalar resolution, Tensor partitionScale, Tensor goal) throws Exception {
    Tensor root = Tensors.vector(8.8, 0.5);
    return createGlcFromRootToGoal(gradientAmp, resolution, partitionScale, root, goal);
  }

  static TrajectoryPlannerContainer createGlcFromRootToGoal(Scalar gradientAmp, RationalScalar resolution, Tensor partitionScale, Tensor root, Tensor goal)
      throws Exception {
    Scalar timeScale = RealScalar.of(60);
    Scalar depthScale = RealScalar.of(100);
    // Tensor partitionScale = Tensors.vector(2e26, 2e26);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Tensor range = Tensors.vector(9, 6.5);
    ImageGradient ipr = new ImageGradient(ResourceData.of("/io/delta_uxy.png"), range, gradientAmp); // -.25 .5
    Scalar maxInput = RealScalar.of(0.1);
    DeltaStateSpaceModel stateSpaceModel = new DeltaStateSpaceModel(ipr, maxInput);
    System.out.println("MaxGradient of field is: " + ipr.maxNormGradient());
    Collection<Flow> controls = DeltaControls.createControls( //
        stateSpaceModel, maxInput, resolution.number().intValue());
    Parameters parameters = new DeltaParameters(resolution, timeScale, depthScale, //
        partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    System.out.println("1/DomainSize: " + parameters.getEta());
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, parameters.getdtMax(), parameters.getTrajectorySize());
    Tensor obstacleImage = ResourceData.of("/io/delta_free.png"); //
    TrajectoryRegionQuery obstacleQuery = //
        SimpleTrajectoryRegionQuery.timeInvariant(new ImageRegion(obstacleImage, range, true));
    DeltaHeuristicGoalManager deltaGoalManager = new DeltaHeuristicGoalManager( //
        goal, Tensors.vector(.3, .3), stateSpaceModel.getMaxPossibleChange());
    // DeltaGoalManager deltaGoalManager = new DeltaGoalManager( //
    // Tensors.vector(2.1, 0.3), Tensors.vector(.3, .3));
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, deltaGoalManager);
    trajectoryPlanner.insertRoot(Tensors.vector(8.8, 0.5));
    return new TrajectoryPlannerContainer(trajectoryPlanner, parameters, stateSpaceModel);
  }

  static TrajectoryPlannerContainer createGlcAny(Scalar gradientAmp, RationalScalar resolution, Tensor partitionScale) throws Exception {
    Scalar timeScale = RealScalar.of(40);
    Scalar depthScale = RealScalar.of(100);
    // Tensor partitionScale = Tensors.vector(6e29, 6e29);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Tensor range = Tensors.vector(9, 6.5);
    ImageGradient ipr = new ImageGradient(ResourceData.of("/io/delta_uxy.png"), range, gradientAmp); // -.25 .5
    Scalar maxInput = RealScalar.of(0.1);
    DeltaStateSpaceModel stateSpaceModel = new DeltaStateSpaceModel(ipr, maxInput);
    System.out.println("MaxGradient of field is: " + ipr.maxNormGradient());
    Collection<Flow> controls = DeltaControls.createControls( //
        stateSpaceModel, maxInput, resolution.number().intValue());
    Parameters parameters = new DeltaParameters(resolution, timeScale, depthScale, //
        partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    System.out.println("1/DomainSize: " + parameters.getEta());
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, parameters.getdtMax(), parameters.getTrajectorySize());
    Tensor obstacleImage = ResourceData.of("/io/delta_free.png");
    TrajectoryRegionQuery obstacleQuery = //
        SimpleTrajectoryRegionQuery.timeInvariant(new ImageRegion(obstacleImage, range, true));
    DeltaHeuristicGoalManager deltaGoalManager = new DeltaHeuristicGoalManager( //
        Tensors.vector(2.9, 2.4), Tensors.vector(.3, .3), stateSpaceModel.getMaxPossibleChange());
    // DeltaGoalManager deltaGoalManager = new DeltaGoalManager( //
    // Tensors.vector(2.1, 0.3), Tensors.vector(.3, .3));
    OptimalAnyTrajectoryPlanner trajectoryPlanner = new OptimalAnyTrajectoryPlanner(parameters.getEta(), stateIntegrator, controls, obstacleQuery,
        deltaGoalManager);
    trajectoryPlanner.switchRootToState((Tensors.vector(8.8, 0.5)));
    return new TrajectoryPlannerContainer(trajectoryPlanner, parameters, stateSpaceModel);
  }
}
