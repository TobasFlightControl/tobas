# Tobas Fail-Safe

## Common Specification

### Fail-Safe Flow

- Disarm if a fail-safe is triggered before takeoff.
- The fail-safe flow is unified and always ends with disarm: RTL -> Land -> Disarm

### Conditions That Do Not Trigger Fail-Safe

1. During manual control
2. When position and velocity estimation accuracy is low

## Behavior

### RTL (Return to Launch)

Ascend to the minimum altitude, return to the arming point, land, and disarm.

### Land

Land in place and disarm.

### Disarm

Stop all motors immediately.

## Check Items

### Battery Fail-Safe

#### Trigger Condition

The battery voltage remains below the threshold for a certain period of time.

#### Behavior

Land

#### Recovery

None

### Radio Fail-Safe

#### Trigger Condition

S.BUS from the receiver is interrupted for a certain period of time.

#### Behavior

- If position can be estimated -> RTL
- If position cannot be estimated -> Land

#### Recovery

Resume manual control once S.BUS is detected again.
