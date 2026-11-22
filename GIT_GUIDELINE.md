# Git Guideline

## Branches

- `${ros-distribution}`:
  Development branch for each ROS distribution.
- `feature/`:
  Branch for feature development.
  Branch from `${ros-distribution}` and delete after merging.
- `hotfix/`:
  Branch for urgent bug fixes.
  Branch from `${ros-distribution}` and delete after merging.

### Example Branch Names

- `jazzy`
- `feature/nonlinear-mpc`
- `hotfix/mag-drivers`

## Commit Message

### Format

```txt
<Type>: <Subject>

<Body>
```

#### Type

- `add`: Add features or files
- `fix`: Fix bugs
- `modify`: Make minor changes or adjustments
- `change`: Change specifications
- `remove`: Remove features or files
- `wip`: Work in progress

#### Subject

- Summary of the commit
- 50 characters or fewer
- Capitalize the first word
- No period at the end
- Use the imperative mood

#### Body (Optional)

- Details of the commit
- 72 characters or fewer per line
- Describe what and why, not how

### Example Commit Messages

- `fix: Resolve price validation bug`

## Pull Request

- Assign Masayoshi Dohi (@Masa0u0) as a reviewer.
