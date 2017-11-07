// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.io.IOException;

import ch.ethz.idsc.owly.demo.rn.R2ImageRegions;
import ch.ethz.idsc.owly.demo.rn.R2xTPolygonStateTimeRegion;
import ch.ethz.idsc.owly.demo.se2.Se2PointsVsRegion;
import ch.ethz.idsc.owly.demo.se2.Se2PointsVsRegions;
import ch.ethz.idsc.owly.demo.util.DemoInterface;
import ch.ethz.idsc.owly.demo.util.ExamplePolygons;
import ch.ethz.idsc.owly.demo.util.TranslationFamily;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Sin;

public class Se2xTLetterDemo implements DemoInterface {
  @Override
  public void start() {
    OwlyAnimationFrame owlyAnimationFrame = new OwlyAnimationFrame();
    Se2xTEntity abstractEntity = new Se2xTEntity(Tensors.vector(6, 6, 1));
    owlyAnimationFrame.set(abstractEntity);
    // ---
    switch (2) {
    case 1: {
      ImageRegion imageRegion = R2ImageRegions.inside_gtob();
      Se2PointsVsRegion se2PointsVsRegion = Se2PointsVsRegions.line(Tensors.vector(.2, .1, 0, -.1), imageRegion);
      TrajectoryRegionQuery trq = SimpleTrajectoryRegionQuery.timeInvariant(se2PointsVsRegion);
      abstractEntity.obstacleQuery = trq;
      owlyAnimationFrame.setObstacleQuery(trq);
      owlyAnimationFrame.addRegionRender(imageRegion);
      break;
    }
    case 2: {
      TranslationFamily shift = new TranslationFamily( //
          scalar -> Tensors.of(Sin.FUNCTION.apply(scalar.multiply(RealScalar.of(0.2))), RealScalar.ZERO));
      Region<StateTime> region = new R2xTPolygonStateTimeRegion( //
          ExamplePolygons.CORNER_TOP_LEFT, shift, () -> abstractEntity.getStateTimeNow().time());
      abstractEntity.obstacleQuery = new SimpleTrajectoryRegionQuery(region);
      owlyAnimationFrame.setObstacleQuery(abstractEntity.obstacleQuery);
      // owlyAnimationFrame.addRegionRender(imageRegion);
      owlyAnimationFrame.addBackground((RenderInterface) region);
      break;
    }
    }
    // ---
    owlyAnimationFrame.configCoordinateOffset(50, 700);
    owlyAnimationFrame.jFrame.setBounds(100, 50, 1200, 800);
    owlyAnimationFrame.jFrame.setVisible(true);
  }

  public static void main(String[] args) throws IOException {
    new Se2xTLetterDemo().start();
  }
}
