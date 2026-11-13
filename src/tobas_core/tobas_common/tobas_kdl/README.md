# tobas_kdl

Extensions for Orocos KDL

## Design Philosophy

- Use error codes consistently for error handling. Avoid memory-heavy exceptions whenever possible.
- Store required allocations, including outputs, in class instance variables whenever possible.
