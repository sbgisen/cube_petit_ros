# cube_petit_shared_controller

Lets **one** PS4 controller, physically (Bluetooth) paired to a single "hub" individual,
drive **whichever** CubePetit individual is currently selected -- switched instantly with a
button press, no Bluetooth re-pairing. Relays over a plain `eclipse-zenoh` session, the same
library `cube_petit_fleet_bridge` uses (see that package's `zenoh_connector.py` for the
established pattern this package follows).

## Why not just re-pair Bluetooth?

Re-pairing the PS4 controller to a different individual takes several seconds each time,
which is disruptive during a demo. This package keeps the controller connected to one "hub"
individual and relays commands to whichever individual is selected instead.

## Roles

### `role:=hub` (one individual, default `cube_petit_orange`)

- `controller_hub_node` subscribes the local `/joy` topic (published by the existing
  `joy_node`, started separately by `cube_petit_bringup/launch/teleop.launch.py`) and detects
  the **rising edge** of `switch_button` (default button `2`, nominally Triangle -- buttons
  `0`/X and `5`/L1 are already used by `ps4.config.yaml`'s `enable_button` /
  `enable_turbo_button`, see that file). Each press cycles to the next name in `robot_names`
  (default `cube_petit_orange,cube_petit_pink`) and publishes it to the `controller/selected_robot`
  zenoh key as `{"robot_name": "..."}`.
- A second, dedicated `teleop_twist_joy_node` instance (started by this package's launch file,
  reusing `cube_petit_bringup/config/ps4.config.yaml`) turns `/joy` into a `Twist` on a
  **local-only** topic (`local_cmd_vel_topic`, default
  `diff_drive_controller/shared_controller/local_cmd_vel`). `controller_hub_node` subscribes
  that topic and relays it to the `controller/cmd_vel` zenoh key as
  `{"linear_x": float, "angular_z": float}`.
- This local topic is deliberately **not** the real `diff_drive_controller/cmd_vel` actuator
  topic: the hub individual also runs `role:=receiver` on itself (see below), and if the relay
  source and the receiver's publish target were the same topic, selecting the hub robot would
  create an infinite hub -> zenoh -> receiver -> hub feedback loop.

**On the hub individual, run both:**

```bash
ros2 launch cube_petit_bringup teleop.launch.py
ros2 launch cube_petit_shared_controller shared_controller.launch.py role:=hub robot_namespace:=cube_petit_orange
```

(or fold the second line into `cube_petit_bringup.launch.py` once this has been validated on
real hardware -- left as a separate launch for now so it's easy to disable independently.)

### `role:=receiver` (every controllable individual, **including** the hub one)

- `controller_receiver_node` subscribes `controller/selected_robot` and `controller/cmd_vel`.
  It resolves its own name via, in order: the `robot_name` launch parameter, the
  `ROBOT_NAMESPACE` environment variable, its own ROS namespace, then falls back to
  `'cube_petit'`.
- Only republishes `controller/cmd_vel` to this individual's own
  `diff_drive_controller/cmd_vel` (`geometry_msgs/msg/TwistStamped`, matching
  `teleop.launch.py`'s `publish_stamped_twist: true`) **while its name matches the current
  selection**. Individuals that are not selected never move.
- The instant this individual **becomes** selected (was not selected, now is), it speaks a
  short "it's me" announcement (`announcement_text`, default `自分だよ!`) through
  `speech_action_server` (`cube_petit_speech_msgs/action/Speech`), the same action
  `cube_petit_fleet_bridge`'s `zenoh_connector.py` `speak` command and
  `cube_petit_bringup`'s `startup_announcer.py` already use. Set `announcement_enabled:=false`
  to disable. Nothing is spoken when a robot is *deselected*.
- The instant this individual **stops** being selected, it publishes one zero-velocity
  `TwistStamped` as a safety stop before going quiet.

**On every controllable individual (orange and pink alike):**

```bash
ros2 launch cube_petit_shared_controller shared_controller.launch.py role:=receiver robot_namespace:=cube_petit_pink
```

**Important:** do **not** also launch `teleop.launch.py` on individuals that only run
`role:=receiver` and have no controller physically attached (no `/dev/input/js0`) -- it would
start a `joy_node` with nothing to read from, for no benefit. `teleop.launch.py` is only needed
on the hub individual.

## Zenoh keys

| Key                          | Published by | Payload                                      |
|-------------------------------|---------------|-----------------------------------------------|
| `controller/selected_robot`   | hub           | `{"robot_name": "cube_petit_pink"}`            |
| `controller/cmd_vel`          | hub           | `{"linear_x": 0.1, "angular_z": -0.5}`         |

These are independent of `cube_petit_fleet_bridge`'s `robots/<robot_name>/...` keys.

## Parameters (see `shared_controller.launch.py` for the full, documented list)

The most important ones for real-robot bring-up:

- `switch_button` (hub, default `2`): **NEEDS REAL-ROBOT VERIFICATION** -- joystick button
  indices depend on the OS/driver mapping and can differ from the nominal PS4 layout. Confirm
  with `ros2 topic echo <joy_topic>` while pressing the intended button, and pass the actual
  index via this parameter if it differs.
- `robot_names` (hub, default `cube_petit_orange,cube_petit_pink`): comma-separated toggle
  order.
- `zenoh_router_endpoint` (both, default `tcp/cube-petit-orange.local:7447`): same default
  router address as `cube_petit_fleet_bridge`.

## Requirements

Same as `cube_petit_fleet_bridge`: the `eclipse-zenoh` pip package, not resolvable via rosdep
on jazzy/noble. See `requirements.txt`:

```bash
pip install --break-system-packages -r requirements.txt
```
