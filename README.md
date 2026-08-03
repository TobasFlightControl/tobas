![Tobas](./docs/docs/assets/brand/logo_black.png#gh-light-mode-only)
![Tobas](./docs/docs/assets/brand/logo_white.png#gh-dark-mode-only)

[![Latest version](https://img.shields.io/github/v/release/TobasFlightControl/tobas)](https://github.com/TobasFlightControl/tobas/releases)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Slack](https://img.shields.io/badge/Slack-User%20Community-4A154B?logo=slack&logoColor=white)](https://join.slack.com/t/tobasusers/shared_invite/zt-43adzzbao-LwcIzacq3aNOAK4F_0olDw)

Tobas is a Linux-based, model-based flight controller for drones and robotic aircraft.
It designs control systems from each airframe's physical model,
so unconventional aircraft can be simulated, configured, and flown through the same ROS 2 interface.

## Quick Links

| Purpose                  | Document                                                                                              |
| ------------------------ | ----------------------------------------------------------------------------------------------------- |
| Use Tobas                | [Tobas User Guide](https://tobasflightcontrol.github.io/tobas/latest/)                                |
| Install Tobas            | [Installation Guide](https://tobasflightcontrol.github.io/tobas/latest/getting_started/installation/) |
| Build from source        | [Setup](./SETUP.md)                                                                                   |
| Contribute changes       | [Contributing to Tobas](./CONTRIBUTING.md)                                                            |
| Edit the documentation   | [Documentation README](./docs/README.md)                                                              |
| Review licensing options | [Commercial License](./COMMERCIAL-LICENSE.md)                                                         |

## Supported Platform

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Debian Trixie for flight-controller images

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

For commercial licensing inquiries, please contact: contact@tobas.jp
