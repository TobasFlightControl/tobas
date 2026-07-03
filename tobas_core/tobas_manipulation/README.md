# tobas_manipulation

## Notes

Joint-space control and task-space control are independent, so it is preferable to implement them as separate nodes.
However, they can be neither serial nor parallel, so the system must guarantee that only one is active.
For that reason, both are implemented in the same node and switched based on the latest command message.
