// code by jl
package ch.ethz.idsc.owly.demo.twd.glc;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.twd.TwdControls;
import ch.ethz.idsc.owly.demo.twd.TwdMinCurvatureGoalManager;
import ch.ethz.idsc.owly.demo.twd.TwdStateSpaceModel;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.HyperplaneRegion;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** if the controls are significantly larger than 1,
 * the gui features long black lines indicating the control vectors */
enum TwdGlcDemo {
  ;
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(6);
    Scalar timeScale = RealScalar.of(32);
    Scalar depthScale = RealScalar.of(20);
    Tensor partitionScale = Tensors.vector(5, 5, 2 * Math.PI / 360 * 20);
    Scalar dtMax = RationalScalar.of(1, 10);
    int maxIter = 2000;
    Scalar wheelDistance = RealScalar.of(0.2);
    Scalar wheelRadius = RealScalar.of(0.05);
    TwdStateSpaceModel stateSpaceModel = new TwdStateSpaceModel(wheelRadius, wheelDistance);
    Parameters parameters = new TwdParameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    parameters.printResolution();
    System.out.println("DomainSize: 1/Eta: " + parameters.getEta().map(n -> RealScalar.ONE.divide(n)));
    // --
    StateIntegrator stateIntegrator = FixedStateIntegrator.createDefault(parameters.getdtMax(), //
        parameters.getTrajectorySize());
    // --
    Collection<Flow> controls = TwdControls.createControls2( //
        stateSpaceModel, parameters.getResolutionInt());
    // --
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new HyperplaneRegion(Tensors.vector(0, -1, 0), RealScalar.of(4)), //
                new HyperplaneRegion(Tensors.vector(0, +1, 0), RealScalar.of(3)) //
            )));
    // --
    Tensor goalCenter = Tensors.vector(2, -2, -1 * Math.PI);
    // TwdDefaultGoalManager goalManager = new TwdDefaultGoalManager(goalCenter, radiusVector);
    TwdMinCurvatureGoalManager goalManager = //
        new TwdMinCurvatureGoalManager(goalCenter, RealScalar.of(0.5), RealScalar.of(50 * Math.PI / 180));
    // --
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner(parameters.getEta(), //
        stateIntegrator, controls, obstacleQuery, goalManager.getGoalInterface());
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0, 0.5 * Math.PI));
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    while (!trajectoryPlanner.getBest().isPresent() && owlyFrame.jFrame.isVisible()) {
      Expand.maxSteps(trajectoryPlanner, 30, parameters.getDepthLimit());
      owlyFrame.setGlc(trajectoryPlanner);
      Thread.sleep(1);
    }
  }
}
