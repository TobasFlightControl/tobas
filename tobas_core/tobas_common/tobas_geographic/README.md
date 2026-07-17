# tobas_geographic

This package delegates geographic coordinate conversions and geomagnetic field calculations to
[GeographicLib](https://geographiclib.sourceforge.io/).

The bundled WMM2025 data files were generated and distributed by the GeographicLib project
from the NOAA World Magnetic Model 2025 coefficients.
WMM2025 is valid from 2025 through 2030.

## Magnetic model data

The files under `data/magnetic` are GeographicLib's native magnetic-model dataset, not the raw `WMM.COF` file published by NOAA.
The GeographicLib project distributes the converted WMM2025 dataset as
[`wmm2025.tar.bz2`](https://sourceforge.net/projects/geographiclib/files/magnetic-distrib/wmm2025.tar.bz2/download).
The underlying model is documented on the
[NOAA World Magnetic Model](https://www.ncei.noaa.gov/products/world-magnetic-model) page.

### Refreshing WMM2025

Install `geographiclib-tools`, which provides `geographiclib-get-magnetic`,
and run the following commands from this package directory:

```sh
geographiclib-get-magnetic -p data -f wmm2025
```

The `-p data` option makes the tool install the dataset in `data/magnetic`.
The `-f` option forces a fresh download and also allows newer models that an older version of the helper script might not list yet.
The command should update both `wmm2025.wmm` and `wmm2025.wmm.cof`.

Review the metadata in `wmm2025.wmm`, especially `Epoch`, `MinTime`, `MaxTime`, `ReleaseDate`, and `DataVersion`,
before committing the updated files.

### Switching to a future WMM release

When a new WMM replaces WMM2025:

1. Download it with `geographiclib-get-magnetic -p data -f <model-name>`.
2. Change the model name passed to `GeographicLib::MagneticModel` in `src/geography.cpp`.
3. Update the model name, source link, and validity period in this README.
4. Remove the superseded dataset after confirming that no consumer still uses it.

The complete set of GeographicLib magnetic datasets is available from the
[GeographicLib magnetic-model distribution](https://sourceforge.net/projects/geographiclib/files/magnetic-distrib/).
