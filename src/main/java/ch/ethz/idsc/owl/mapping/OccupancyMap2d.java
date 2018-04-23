// code by ynager
package ch.ethz.idsc.owl.mapping;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.awt.image.WritableRaster;
import java.util.Collection;
import java.util.Optional;
import java.util.stream.Collectors;

import ch.ethz.idsc.owl.data.nd.NdCenterInterface;
import ch.ethz.idsc.owl.data.nd.NdCluster;
import ch.ethz.idsc.owl.data.nd.NdEntry;
import ch.ethz.idsc.owl.data.nd.NdMap;
import ch.ethz.idsc.owl.data.nd.NdTreeMap;
import ch.ethz.idsc.owl.gui.AffineTransforms;
import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.math.region.BoundedBoxRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Ceiling;
import ch.ethz.idsc.tensor.sca.Floor;
import ch.ethz.idsc.tensor.sca.Sign;

public class OccupancyMap2d implements RenderInterface {
  private static final int MAX_DEPTH = 7;
  private static final int MAX_DENSITY = 6;
  /** red channel is used for occupancy check */
  private static final int OCCUPIED_CHANNEL = 0xFF << 16;
  private static final int OCCUPIED_COLOR = 0x7FFF << 16;
  private static final int EMPTY_COLOR = 0x1000FF << 8;
  // ---
  private final Tensor lbounds;
  private final Scalar gridRes;
  private final Scalar gridInv;
  /** nd map is used to obtain nearest neighbors */
  private final NdMap<Scalar> ndMap;
  /** buffered image is used to check for occupancy and visualization */
  private final BufferedImage bufferedImage;
  private final int[] pixels;
  private final Region<Tensor> mapRegion;

  /** @param lbounds vector of length 2
   * @param ubounds vector of length 2
   * @param gridRes */
  public OccupancyMap2d(Tensor lbounds, Tensor ubounds, Scalar gridRes) {
    ndMap = new NdTreeMap<>(lbounds, ubounds, MAX_DENSITY, MAX_DEPTH); // magic const
    this.lbounds = lbounds;
    this.gridRes = Sign.requirePositive(gridRes);
    gridInv = gridRes.reciprocal();
    Tensor size = Ceiling.of(ubounds.subtract(lbounds).multiply(gridInv));
    bufferedImage = new BufferedImage( //
        size.Get(0).number().intValue(), //
        size.Get(1).number().intValue(), //
        BufferedImage.TYPE_INT_ARGB);
    WritableRaster writableRaster = bufferedImage.getRaster();
    DataBufferInt dataBufferByte = (DataBufferInt) writableRaster.getDataBuffer();
    pixels = dataBufferByte.getData();
    mapRegion = new BoundedBoxRegion(ubounds.subtract(lbounds).divide(RealScalar.of(2)) //
        .add(lbounds), ubounds.subtract(lbounds).divide(RealScalar.of(2)));
    Graphics graphics = bufferedImage.getGraphics();
    graphics.setColor(new Color(EMPTY_COLOR, true));
    graphics.fillRect(0, 0, bufferedImage.getWidth(), bufferedImage.getHeight());
  }

  public synchronized boolean insert(Tensor pos) {
    if (isOccupied(pos))
      return false;
    Tensor tile = toTile(pos); // get tile corresponding to position
    ndMap.add(tile, RealScalar.of(1));
    drawTile(tile);
    return true;
  }

  public boolean remove(Tensor pos) {
    // TODO: build this method
    return false;
  }

  public final int getTreeSize() {
    return ndMap.size();
  }

  public synchronized boolean isOccupied(Tensor pos) {
    if (!mapRegion.isMember(pos))
      return true;
    Tensor tile = toTile(pos);
    Scalar idx = (tile.Get(1)).multiply(RealScalar.of(bufferedImage.getWidth())).add(tile.Get(0));
    return ((int) (pixels[idx.number().intValue()] & OCCUPIED_CHANNEL) != 0) ? true : false;
  }

  // calculates L2 distance between center of respective query tile and closest obstacle tile
  public synchronized Scalar getL2DistToClosest(Tensor pos) {
    NdCenterInterface distanceInterface = NdCenterInterface.euclidean(toTile(pos));
    NdCluster<Scalar> cluster = ndMap.buildCluster(distanceInterface, 1);
    Optional<NdEntry<Scalar>> closest = cluster.stream().findFirst();
    if (closest.isPresent())
      return closest.get().distance().multiply(gridRes);
    return DoubleScalar.POSITIVE_INFINITY;
  }

  public synchronized Collection<Scalar> getKNearest(Tensor pos, int k_nearest) {
    NdCenterInterface distanceInterface = NdCenterInterface.euclidean(pos);
    NdCluster<Scalar> cluster = ndMap.buildCluster(distanceInterface, k_nearest);
    // System.out.println("considered " + cluster.considered() + " " + ndMap.size());
    return cluster.stream().map(NdEntry::value).collect(Collectors.toList());
  }

  public OccupancyMap2d getCopy() throws CloneNotSupportedException {
    return (OccupancyMap2d) this.clone();
  }

  /** @param state of which only the first two entries are considered
   * @return */
  /* package for testing */ Tensor toTile(Tensor state) {
    return Floor.of(state.extract(0, 2).subtract(lbounds).multiply(gridInv));
  }

  private void drawTile(Tensor tile) {
    int xvalue = tile.Get(0).number().intValue();
    int yvalue = tile.Get(1).number().intValue();
    bufferedImage.setRGB(xvalue, yvalue, OCCUPIED_COLOR); // set red channel to 0xFF
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    final Tensor matrix = geometricLayer.getMatrix();
    AffineTransform affineTransform = AffineTransforms.toAffineTransform(matrix);
    double scale = gridRes.number().doubleValue();
    affineTransform.scale(scale, scale);
    affineTransform.translate(lbounds.Get(0).multiply(gridInv).number().doubleValue(), //
        lbounds.Get(1).multiply(gridInv).number().doubleValue());
    graphics.drawImage(bufferedImage, affineTransform, null);
  }
}
