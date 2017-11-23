// code by jph
package ch.ethz.idsc.owly.demo.se2.twd;

import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.math.map.RigidFamily;
import ch.ethz.idsc.owl.math.map.Se2Family;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owl.sim.CameraEmulator;
import ch.ethz.idsc.owl.sim.LidarEmulator;
import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.rn.R2xTImageStateTimeRegion;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;

/** the obstacle region in the demo is the outside of a rotating letter 'a' */
public class TwdxTImageAnimationDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    TwdDuckieFlows twdConfig = new TwdDuckieFlows(RealScalar.of(1.2), RealScalar.of(.5));
    TwdxTEntity twdxTEntity = new TwdxTEntity(twdConfig, new StateTime(Tensors.vector(-1, -1, 1.0), RealScalar.ZERO));
    owlyAnimationFrame.set(twdxTEntity);
    // ---
    RigidFamily rigidFamily = Se2Family.rotationAround( //
        Tensors.vectorDouble(1.5, 2), time -> time.multiply(RealScalar.of(0.1)));
    ImageRegion imageRegion = R2ImageRegions.inside_circ();
    Region<StateTime> region = new R2xTImageStateTimeRegion( //
        imageRegion, rigidFamily, () -> twdxTEntity.getStateTimeNow().time());
    // ---
    TrajectoryRegionQuery trq = new SimpleTrajectoryRegionQuery(region);
    {
      RenderInterface renderInterface = new CameraEmulator( //
          48, RealScalar.of(10), twdxTEntity::getStateTimeNow, trq);
      owlyAnimationFrame.addBackground(renderInterface);
    }
    {
      RenderInterface renderInterface = new LidarEmulator( //
          LidarEmulator.DEFAULT, () -> twdxTEntity.getStateTimeNow(), trq);
      owlyAnimationFrame.addBackground(renderInterface);
    }
    owlyAnimationFrame.setObstacleQuery(trq);
    owlyAnimationFrame.addBackground((RenderInterface) region);
    owlyAnimationFrame.configCoordinateOffset(200, 400);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) {
    new TwdxTImageAnimationDemo().start();
  }
}
