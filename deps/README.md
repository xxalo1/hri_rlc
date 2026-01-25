
# Dependencies

`deps/manifests/` contains `vcstool` (`vcs`) manifests that define the external repositories this workspace depends on.

## Requirements

Install `vcstool`:

- Ubuntu/Debian: `sudo apt-get update && sudo apt-get install -y python3-vcstool`
- pip: `python3 -m pip install vcstool`

## Import

From the repository root:

- Clone all dependencies: `vcs import < deps/manifests/dependency.repos`

## Update

From the repository root (after import):

- Pull updates for all imported repos: `vcs pull`
- Check status: `vcs status`

## Nested dependency manifests

Some upstream repos (e.g. `franka_ros2`) ship their own `.repos` file.
Recommendation: vendor those repositories into our `deps/manifests/*.repos` so this repo has a single, reproducible import step.

## Scripts and patches

Automation for multi-manifest import/update and patch application will live under `deps/scripts/` and `patches/`.

