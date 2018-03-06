// code by jph and jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.Collection;

import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.any.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.glc.par.Parameters;
import ch.ethz.idsc.owl.glc.std.StandardTrajectoryPlanner;
import ch.ethz.idsc.owl.gui.ren.VectorFieldRender;
import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.owl.math.VectorFields;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.sample.BoxRandomSample;
import ch.ethz.idsc.owl.math.sample.RandomSample;
import ch.ethz.idsc.owl.math.sample.RandomSampleInterface;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.delta.DeltaAltStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.DeltaFlows;
import ch.ethz.idsc.owly.demo.delta.DeltaHeuristicGoalManager;
import ch.ethz.idsc.owly.demo.delta.DeltaParameters;
import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.demo.rn.EuclideanDistanceDiscoverRegion;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.ResourceData;

public enum DeltaHelper {
  ;
  // ---
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
    Scalar timeScale = RealScalar.of(50);
    Scalar depthScale = RealScalar.of(100);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Tensor range = Tensors.vector(9, 6.5);
    ImageGradient ipr = ImageGradient.linear(ResourceData.of("/io/delta_uxy.png"), range, gradientAmp);
    Scalar maxInput = RealScalar.of(0.1);
    DeltaAltStateSpaceModel stateSpaceModel = new DeltaAltStateSpaceModel(ipr, maxInput);
    System.out.println("MaxGradient of field is: " + ipr.maxNormGradient());
    Collection<Flow> controls = new DeltaFlows(stateSpaceModel, maxInput).getFlows( //
        resolution.number().intValue());
    Parameters parameters = new DeltaParameters(resolution, timeScale, depthScale, //
        partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    System.out.println("1/DomainSize: " + parameters.getEta());
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, parameters.getdtMax(), parameters.getTrajectorySize());
    Tensor obstacleImage = ResourceData.of("/io/delta_free.png"); //
    TrajectoryRegionQuery obstacleQuery = //
        SimpleTrajectoryRegionQuery.timeInvariant(new ImageRegion(obstacleImage, range, true));
    DeltaHeuristicGoalManager deltaGoalManager = new DeltaHeuristicGoalManager( //
        goal, Tensors.vector(0.3, 0.3), stateSpaceModel.getMaxPossibleChange());
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, deltaGoalManager);
    trajectoryPlanner.insertRoot(new StateTime(Tensors.vector(8.8, 0.5), RealScalar.ZERO));
    return new TrajectoryPlannerContainer(trajectoryPlanner, parameters, stateSpaceModel);
  }

  static TrajectoryPlannerContainer createGlcAny(Scalar gradientAmp, RationalScalar resolution, Tensor partitionScale) throws Exception {
    Scalar timeScale = RealScalar.of(50);
    Scalar depthScale = RealScalar.of(100);
    Scalar dtMax = RationalScalar.of(1, 6);
    Tensor goal = Tensors.vector(2.9, 2.4);
    Tensor root = Tensors.vector(8.8, 0.5);
    int maxIter = 2000;
    Tensor range = Tensors.vector(9, 6.5);
    ImageGradient ipr = ImageGradient.linear(ResourceData.of("/io/delta_uxy.png"), range, gradientAmp);
    Scalar maxInput = RealScalar.of(0.1);
    DeltaAltStateSpaceModel stateSpaceModel = new DeltaAltStateSpaceModel(ipr, maxInput);
    System.out.println("MaxGradient of field is: " + ipr.maxNormGradient());
    Collection<Flow> controls = new DeltaFlows(stateSpaceModel, maxInput).getFlows( //
        resolution.number().intValue());
    Parameters parameters = new DeltaParameters(resolution, timeScale, depthScale, //
        partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    System.out.println("1/DomainSize: " + parameters.getEta());
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, parameters.getdtMax(), parameters.getTrajectorySize());
    Tensor obstacleImage = ResourceData.of("/io/delta_free.png");
    // TrajectoryRegionQuery obstacleQuery = //
    // SimpleTrajectoryRegionQuery.timeInvariant(new ImageRegion(obstacleImage, range, true));
    Scalar sensingRadius = RealScalar.of(3);
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant( //
        EuclideanDistanceDiscoverRegion.of(new ImageRegion(obstacleImage, range, true), root, sensingRadius));
    DeltaHeuristicGoalManager deltaGoalManager = new DeltaHeuristicGoalManager( //
        goal, Tensors.vector(0.3, 0.3), stateSpaceModel.getMaxPossibleChange());
    OptimalAnyTrajectoryPlanner trajectoryPlanner = new OptimalAnyTrajectoryPlanner(parameters.getEta(), stateIntegrator, controls, obstacleQuery,
        deltaGoalManager);
    trajectoryPlanner.switchRootToState(new StateTime(root, RealScalar.ZERO));
    return new TrajectoryPlannerContainer(trajectoryPlanner, parameters, stateSpaceModel);
  }

  public static VectorFieldRender vectorFieldRender(StateSpaceModel stateSpaceModel, Tensor range, Region<Tensor> region, Scalar factor) {
    VectorFieldRender vectorFieldRender = new VectorFieldRender();
    RandomSampleInterface sampler = new BoxRandomSample(Tensors.vector(0, 0), range);
    Tensor points = Tensor.of(RandomSample.of(sampler, 1000).stream().filter(p -> !region.isMember(p)));
    vectorFieldRender.uv_pairs = //
        VectorFields.of(stateSpaceModel, points, DeltaEntity.FALLBACK_CONTROL, factor);
    return vectorFieldRender;
  }
}
