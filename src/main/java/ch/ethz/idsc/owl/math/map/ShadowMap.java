package ch.ethz.idsc.owl.math.map;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.util.function.Supplier;

import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.math.region.ImageArea;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.sim.LidarEmulator;
import ch.ethz.idsc.owly.demo.util.RegionRenders;

public class ShadowMap implements RenderInterface {
  //
  private BufferedImage bufferedImage;
  private final LidarEmulator lidar;
  private final Supplier<StateTime> supplier;
  private final Rectangle2D rInit;
  private Area shadowArea;
  private final Area obstacleArea;
  private int counter = 0;
  //
  private static final Color COLOR_SHADDOW_FILL = new Color(255, 50, 74, 16);
  private static final Color COLOR_SHADDOW_DRAW = new Color(255, 50, 74, 64);

  public ShadowMap(LidarEmulator lidar, ImageRegion imageRegion, Supplier<StateTime> supplier) {
    this.lidar = lidar;
    this.supplier = supplier;
    this.bufferedImage = RegionRenders.image(imageRegion.image());
    // turn imageRegion into area
    ImageArea imageArea = new ImageArea(bufferedImage, new Color(244, 244, 244), 5);
    AffineTransform tx = new AffineTransform();
    tx.scale(3.6, 3.6);
    tx.translate(13.6, -5.7); // TODO: get transformation from imageRegion
    obstacleArea = imageArea.createTransformedArea(tx);
    Stroke stroke = new BasicStroke(4, BasicStroke.CAP_ROUND, BasicStroke.JOIN_BEVEL);
    Shape strokeShape = stroke.createStrokedShape(obstacleArea);
    obstacleArea.add(new Area(strokeShape));
    // define initial shadow area
    this.rInit = new Rectangle2D.Double();
    this.rInit.setFrame(50, 0, 720, 700); // TODO: somehow get frame size from imageRegion
    this.shadowArea = new Area(rInit);
    // subtract obstacles from shaddow
    shadowArea.subtract(obstacleArea);
  }

  public BufferedImage getBufferedImage() {
    return bufferedImage;
  }

  public void updateMap(Path2D lidarPath2D) {
    // subtract current lidar measurement from shadow
    shadowArea.subtract(new Area(lidarPath2D));
    // inflate shadow area
    // TODO: calculate stroke based on v_max of pedestrian and passed time
    Stroke stroke = new BasicStroke(1, BasicStroke.CAP_ROUND, BasicStroke.JOIN_BEVEL);
    Shape strokeShape = stroke.createStrokedShape(shadowArea);
    shadowArea.add(new Area(strokeShape));
    shadowArea.subtract(obstacleArea);
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Se2Bijection se2Bijection = new Se2Bijection(supplier.get().state());
    geometricLayer.pushMatrix(se2Bijection.forward_se2());
    Path2D lidarPath2D = geometricLayer.toPath2D(lidar.getPolygon());
    geometricLayer.popMatrix();
    //
    // TODO: replace counter with time reference passed to updateMap
    // TODO: move map calculation to new thread, separate from render()
    if (counter == 0) {
      updateMap(lidarPath2D);
    }
    counter = (counter + 1) % 2;
    //
    // draw
    graphics.setColor(COLOR_SHADDOW_FILL);
    graphics.fill(shadowArea);
    graphics.setColor(COLOR_SHADDOW_DRAW);
    graphics.draw(shadowArea);
    //
  }
}
