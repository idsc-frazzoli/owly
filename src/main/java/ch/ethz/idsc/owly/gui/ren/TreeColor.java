// code by jph
package ch.ethz.idsc.owly.gui.ren;

import ch.ethz.idsc.owly.gui.ColorLookup;

/** helper class to adapt the node and edge colors of a tree
 * to the dimension of the statespace */
enum TreeColor {
  LO(0.5, 0.300), //
  HI(0.2, 0.075), //
  ;
  // ---
  public static TreeColor ofDimensions(int dims) {
    return dims <= 2 ? LO : HI;
  }

  // ---
  public final ColorLookup nodeColor;
  public final ColorLookup edgeColor;

  private TreeColor(double node_alpha, double edge_alpha) {
    nodeColor = ColorLookup.hsluv_lightness(.50, node_alpha);
    edgeColor = ColorLookup.hsluv_lightness(.65, edge_alpha);
  }
}
