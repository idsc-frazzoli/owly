// code by jph
package ch.ethz.idsc.owly.demo.se2.twd;

import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.rn.R2xTImageStateTimeRegion;
import ch.ethz.idsc.owly.demo.util.CameraEmulator;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.LidarEmulator;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.se2.RigidFamily;
import ch.ethz.idsc.owly.math.se2.Se2Family;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;

/** the obstacle region in the demo is the outside of a rotating letter 'a' */
public class TwdxTImageAnimationDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    TwdConfig twdConfig = new TwdConfig(RealScalar.of(1.2), RealScalar.of(.5));
    TwdxTEntity twdxTEntity = new TwdxTEntity(twdConfig, Tensors.vector(-1, -1, 1.0));
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
          48, RealScalar.of(10), () -> twdxTEntity.getStateTimeNow(), trq);
      owlyAnimationFrame.addBackground(renderInterface);
    }
    {
      RenderInterface renderInterface = new LidarEmulator( //
          LidarEmulator.DEFAULT, RealScalar.of(10), () -> twdxTEntity.getStateTimeNow(), trq);
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
