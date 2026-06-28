# vendor/ (tracked)

Vendor-supplied material that is safe and useful to keep in the repository so the
project is self-contained.

## Layout

- `vendor/docs/` - vendor technical documentation and reference:
  - `UM001 - 3DSS-DX iDX Sonar Manual - 2.3A.pdf` - full sonar manual.
  - `UM002 - 3DSS-IDX Quick Start Guide - 1.8.pdf` - setup guide; **the
    authoritative source for the network topology, IP addressing, and ports**
    (see §4 Network Setup and §12 Third Party Acquisition Software).
  - `UM005 - 3DSS-DX AUV Integration Guide - 1.4.pdf` - OEM/AUV integration.
  - `ASV_Integration 1.1.pdf`
  - `3DSS-iDX-450 Overview and Offsets.pdf` - sensor positions / lever arms.
  - `3dss-dx-control-interface/` - DX Control Command Interface guide (TCP 23840).
  - `3dss-dx-struct-api-0.6/` - 3DSS-DX struct API (C++ header, PDF, CHM,
    license) - the binary stream format the driver parses (TCP 23848).
  - `drawings/` - mechanical mount drawings (PNG0027-* flange/stem mounts).
- `vendor/software/` - 3DSS PC software installers (Windows):
  - `3dss-dx-control-*-install.exe` - sonar control application
  - `3dss-dx-display-*-install.exe` - sonar display/visualisation application

## Related locations

- `docs/` - our own package documentation (architecture, sonar control,
  SBG integration, etc.). Vendor-supplied material does NOT go there.
- `local/` (gitignored) - KTH-specific and irrelevant material; see
  `local/README.md`.

> Note: the installers are tracked as plain git blobs (~26 MB total). Avoid
> dropping large raw dumps or datasets here - those belong in `local/`.
