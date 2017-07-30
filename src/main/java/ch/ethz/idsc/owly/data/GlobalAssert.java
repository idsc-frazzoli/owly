// code by jph
package ch.ethz.idsc.owly.data;

/** placements of class not final */
public enum GlobalAssert {
  ;
  public static void that(boolean valid) {
    if (!valid)
      throw new RuntimeException();
  }
}
