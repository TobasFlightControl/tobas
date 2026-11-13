# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Tobas, Inc.

# tobas::kdl::RigidBodyInertia
float64 mass                            # [kg] Mass
tobas_kdl_msgs/Vector cog               # [m] Center of gravity
tobas_kdl_msgs/RotationalInertia i_cog  # [kg * m^2] Moment of inertia wrt. CoG
