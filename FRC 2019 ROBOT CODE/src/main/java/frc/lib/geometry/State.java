package frc.lib.geometry;

import frc.lib.Util.CSVWritable;
import frc.lib.Util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
