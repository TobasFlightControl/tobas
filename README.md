# Tobas - Flight Controller for All Drones

Tobas is a model-based flight controller for drones and robotic aircraft.
It uses each airframe's physical properties when designing the control system,
so unconventional aircraft can be simulated, configured, and flown with the same ROS 2 based interface.

## Quick Links

| Purpose                  | Link                                                                                                  |
| ------------------------ | ----------------------------------------------------------------------------------------------------- |
| Use Tobas                | [Tobas User Guide](https://tobasflightcontrol.github.io/tobas/latest/)                                |
| Install Tobas            | [Installation Guide](https://tobasflightcontrol.github.io/tobas/latest/getting_started/installation/) |
| Build from source        | [Contributing to Tobas](./CONTRIBUTING.md)                                                            |
| Edit the documentation   | [Documentation README](./docs/README.md)                                                              |
| Review licensing options | [Commercial License](./COMMERCIAL-LICENSE.md)                                                         |

## Supported Platform

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Debian Trixie for flight-controller images

See the installation guide linked above for the full setup procedure.

## Repository Layout

- `docs`: MkDocs-based user and developer documentation.
- `tobas_core`: Core flight-control, estimation, hardware, message, failsafe, and utility packages.
- `tobas_gui`: Setup, ground-station, simulation, tuning, and visualization tools.
- `tobas_examples`: Example packages and code-style references.
- `tobas_dev_tools`: Development, synchronization, and deployment helper scripts.
- `tobas_deb`: Debian packaging resources for supported images.
- `tobas_external`: Third-party libraries wrapped for the Tobas workspace.

## For Contributors

See [Contributing to Tobas](./CONTRIBUTING.md) for source setup, code style, pre-commit checks, and Git guidelines.

## License

Unless otherwise noted, this repository is licensed under the GNU General Public License,
version 3 or any later version (GPL-3.0-or-later).

The `*_msgs` packages, including their `.msg`, `.srv`, and `.action` files,
are licensed under Apache-2.0.

See [LICENSE](./LICENSE) for the default open source license,
and [LICENSES/Apache-2.0.txt](./LICENSES/Apache-2.0.txt) for the Apache-2.0 license text.

If you want to distribute Tobas as part of a proprietary product,
or if you do not wish to comply with the GPL for distribution,
alternative commercial licensing is available from Tobas.

See [COMMERCIAL-LICENSE.md](./COMMERCIAL-LICENSE.md) for commercial licensing information.

For commercial licensing inquiries, please contact: m.dohi@tobas.jp
