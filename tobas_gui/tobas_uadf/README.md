# Universal Aircraft Description Format

## Specification

Aircraft extension for XACRO.
Describes the parts related to the aircraft equations of motion, up to force computation.

### Special Joints

- `thrust`: propulsion unit
  - `direction`: rotation direction
    - `value`: `cw`/`ccw`
- `cs`: fixed-wing control surface
- `tilt`: tilt joint
