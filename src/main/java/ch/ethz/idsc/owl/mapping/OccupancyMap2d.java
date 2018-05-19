// code by ynager
package ch.ethz.idsc.owl.mapping;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.awt.image.WritableRaster;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

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
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Mean;
import ch.ethz.idsc.tensor.sca.Ceiling;
import ch.ethz.idsc.tensor.sca.Floor;
import ch.ethz.idsc.tensor.sca.Sign;

/** all pixels have the same amount of weight or clearance radius attached */
public class OccupancyMap2d implements MappingInterface, Region<Tensor>, RenderInterface {
  private static final int MAX_DEPTH = 7;
  private static final int MAX_DENSITY = 6;
  /** red channel is used for occupancy check */
  private static final int OCCUPIED_CHANNEL = 0xFF << 16;
  private static final int OCCUPIED_COLOR = 0x7FFF << 16;
  private static final int EMPTY_COLOR = 0x1000FF << 8;
  // ---
  // private static final Scalar DEFAULT_COST = DoubleScalar.of(1);
  // ---
  private final Tensor lbounds;
  private final Scalar gridRes;
  private final Scalar gridInv;
  /** nd map is used to obtain nearest neighbors */
  private final NdMap<Void> ndMap;
  /** buffered image is used to check for occupancy and visualization */
  // TODO YN LONGTERM due to the collection/list::remove call i would only insert
  // exact precision values into occupiedList, which at the moment doesn't happen
  // also, what is the role of the 3rd coordinate... under certain circumstances the remove doesn't work.
  // therefore i have removed the 3rd coordinate for now
  private final List<Tensor> occupiedList;
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
    occupiedList = new ArrayList<>();
    Tensor center = Mean.of(Tensors.of(lbounds, ubounds));
    mapRegion = new BoundedBoxRegion(center, ubounds.subtract(center));
    Graphics graphics = bufferedImage.getGraphics();
    graphics.setColor(new Color(EMPTY_COLOR, true));
    graphics.fillRect(0, 0, bufferedImage.getWidth(), bufferedImage.getHeight());
  }

  /** Build the NdTree structure by inserting all tiles in @param occupiedTiles */
  private void buildNdTree(List<Tensor> occupiedTiles) {
    ndMap.clear();
    for (Tensor i : occupiedTiles)
      ndMap.add(i, null); // .extract(0, 2), i.Get(2)
  }

  /** Insert an occupied tile at @param state */
  public synchronized boolean insert(Tensor state) {
    if (isMember(state))
      return false;
    final Tensor tile = toTile(state); // get tile corresponding to state
    drawTile(tile);
    return occupiedList.add(tile);
  }

  /** Remove an occupied tile @param state */
  public boolean remove(Tensor state) {
    final Tensor tile = toTile(state);
    return occupiedList.remove(tile);
  }

  /** Get the size of the NdTree since last updated
   * @return tree size */
  public final int getTreeSize() {
    return ndMap.size();
  }

  /** Check if tile at @param state is occupied */
  @Override
  public boolean isMember(Tensor state) {
    if (state.length() != 2)
      state = state.extract(0, 2); // TODO truncation may not needed if only state.Get(0) and state.Get(1) are used
    if (!mapRegion.isMember(state))
      return true;
    final Tensor tile = toTile(state);
    final Scalar idx = (tile.Get(1)).multiply(RealScalar.of(bufferedImage.getWidth())).add(tile.Get(0));
    return ((pixels[idx.number().intValue()] & OCCUPIED_CHANNEL) != 0) ? true : false;
  }

  /** returns the L2 distance between the center of the tile corresponding to @param state
   * and closest occupied state
   * @return distance */
  public Scalar getL2DistToClosest(Tensor state) {
    if (state.length() != 2)
      state = state.extract(0, 2);
    NdCenterInterface distanceInterface = NdCenterInterface.euclidean(toTile(state));
    NdCluster<Void> cluster = ndMap.buildCluster(distanceInterface, 1);
    Optional<NdEntry<Void>> closest = cluster.stream().findFirst();
    // System.out.print(ndMap.size() + " " + ndMapQuery.size() + "\n");
    if (closest.isPresent())
      return closest.get().distance().multiply(gridRes);
    return DoubleScalar.POSITIVE_INFINITY;
  }

  /** @param state of which only the first two entries are considered
   * @return tile indices */
  /* package for testing */ Tensor toTile(Tensor state) {
    return Floor.of(state.extract(0, 2).subtract(lbounds).multiply(gridInv));
  }

  /** draws an occupied @param tile into bufferedImage */
  private void drawTile(Tensor tile) {
    final int xvalue = tile.Get(0).number().intValue();
    final int yvalue = tile.Get(1).number().intValue();
    bufferedImage.setRGB(xvalue, yvalue, OCCUPIED_COLOR);
  }

  @Override // from AbstractMap
  public void prepareForQuery() {
    buildNdTree(occupiedList);
  }

  @Override // from Renderinterface
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    final Tensor matrix = geometricLayer.getMatrix();
    AffineTransform affineTransform = AffineTransforms.toAffineTransform(matrix);
    final double scale = gridRes.number().doubleValue();
    affineTransform.scale(scale, scale);
    affineTransform.translate( //
        lbounds.Get(0).multiply(gridInv).number().doubleValue(), //
        lbounds.Get(1).multiply(gridInv).number().doubleValue());
    graphics.drawImage(bufferedImage, affineTransform, null);
  }
}
