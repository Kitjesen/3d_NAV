# sim/launch Boundary

`sim/launch/` is a Legacy ROS launch / smoke contract. Keep these files stable
for compatibility with older smoke workflows, but do not treat them as the
canonical server-side simulation closure path. Current G4 evidence is collected
through `sim/scripts/server_sim_closure.py` and the reports under
`artifacts/server_sim_closure/`.

## Safety Rules

- Use an isolated ROS_DOMAIN_ID for every run.
- Do not run on a robot ROS domain.
- The selected domain must not have hardware cmd_vel subscribers on `/cmd_vel`,
  `/nav/cmd_vel`, or equivalent robot driver command topics.
- Do not source robot-side service launch environments that attach physical
  drivers.
- Do not use these launch files as proof of PCT, Fast-LIO2, saved-map
  relocalization, or physical gait readiness unless a stricter gate consumes
  and validates their report.

## Current Role

- `sim.launch.py` preserves a historical ROS launch entrypoint.
- `sim_full.sh` is a manual smoke helper that may source ROS and install spaces.
- Prefer explicit gate scripts in `sim/scripts/` for repeatable evidence.
