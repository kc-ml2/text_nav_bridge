# text_nav_bridge

`text_nav_bridge` converts free-form text navigation commands into Nav2 goals.

This package is the navigation-side bridge in the text navigation pipeline:

- Loads text landmarks produced by [TextMap](https://github.com/kc-ml2/TextMap)
- Matches a user text query against landmark text entries
- Chooses a reachable goal near the matched landmark from the current Nav2 map
- Sends that goal to Nav2 through `nav2_msgs/action/NavigateToPose`

This README only summarizes the package role and ROS interface.
For dependencies, build instructions, and end-to-end launch examples, see the
[text_navigation](../../../text_navigation/) example repository.

## Upstream Inputs

This package expects the following inputs from the upstream text navigation pipeline:

- Landmark file from [TextMap](https://github.com/kc-ml2/TextMap): `landmarks.yaml`
- A Nav2-compatible map or localization source, such as `map.pgm`, `map.yaml` or `rtabmap.db`

For the full pipeline, setup details, and runnable examples, see the
[text_navigation](../../../text_navigation/) example repository.

## Interface

### Interface with Nav2

`text_nav_bridge` acts as a Nav2 client.

- Action client:
  `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)
- Map input used for goal selection:
  `/map` (`nav_msgs/msg/OccupancyGrid`)

The node does not perform path planning itself.
It selects a reachable goal pose near the landmark and hands the request to Nav2.

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/text_nav/command` | `std_msgs/msg/String` | Free-form navigation command such as `"restroom"` |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Occupancy map used to find a reachable goal near the matched landmark |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/text_nav/status` | `std_msgs/msg/String` | Navigation and error status |
| `/text_nav/goal_marker` | `visualization_msgs/msg/Marker` | Selected navigation goal marker for RViz |
| `/textmap/markers` | `visualization_msgs/msg/MarkerArray` | Republished landmark markers loaded from the landmark YAML |

## License

Apache License 2.0
