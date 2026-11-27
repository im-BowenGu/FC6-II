# Movable-fin CFD roadmap

This directory defines the **interface** for a future moving-fin/canard
simulation. It is deliberately not required by the PR gate: the current
verification uses a frozen static-force lookup (see `../sim/`), which is
fast, deterministic and sufficient to detect regressions in the flight
software.

## Staged plan

1. **Geometry (done).** One fin plate STL, `../cad/fin_plate.stl`, generated
   by `../cad/generate_fin_plate_stl.py`. The hinge is at `x = 0`; the plate
   spans `+y`. The rocket envelope STL and 20k-step freeform-fin geometry are
   in the main rocket project (`rocket/cfd/fluidx3d/UKROC_2027_cfd.stl`).
2. **State table (done).** `fin_states.yaml` lists the four fin angles for
   every existing CFD case and a proposed intermediate sweep. Each state can
   reference either a separate STL (`<state>.stl`) or a rotation of the one
   plate about the hinge.
3. **Static sweep (current).** Run FluidX3D with FP16S (D3Q19/SRT) in an
   axis-aligned box around the rocket, 26 m/s freestream, and record force,
   torque and Cd per state. Fit only the 15° cases; keep 4°/8° as validation
   because a 4° plate is below the 5.38 mm voxel scale.
4. **Coupled surrogate (current).** Feed the fitted lookup and the OpenRocket
   thrust/mass/density history into `../sim/run_dynamic_sim.py`, which runs
   the exact 100 Hz guidance loop against the plant and checks apogee ±5 m.
5. **Moving-boundary CFD (future SDK + production task).** Re-mesh the fin
   per control step, or use an immersed-boundary representation, and run the
   firmware loop against the flow solver in lockstep with motor burn time.
   This is a separate simulator product and is not a prerequisite for an
   additive firmware PR.

## Why this split

A moving-boundary solve is expensive and changes with every knob in the flow
solver. Keeping the frozen-lookup regression gate independent means a PR can
be verified as harmless quickly, while the higher-fidelity model evolves in
its own release cycle. When the moving-boundary solver is available, replace
`../sim/data/cfd/fluidx3d/logs/` with its outputs and keep the same
`--check` assertions.

## Open hardware questions

* Confirm the physical fin numbering against the X-mix: `xmix_p15` couples
  pitch and yaw rather than producing pure pitch.
* Measure fin hinge friction, servo lag and flutter on the bench; the
  simulator uses a first-order 35 ms lag and analytic damping.
