# Tobas GUI

## Design Philosophy

- In principle, separate apps by function
- The purpose of a single app should be explainable in one line
- Split GUIs when doing so improves usability
  - The usage order is fixed (e.g. STM32CubeMX -> STM32CubeIDE)
  - Usage frequency differs
  - They may be used at the same time

## Sizing Rules

- Use multiples of 8 pixels for widget dimensions, margins, padding, and spacing whenever possible.
- When multiples of 8 are impractical, use multiples of 4 pixels.
- Values below 4 pixels may be used for fine details such as borders.
