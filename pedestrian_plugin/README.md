# pedestrian_plugin

- Pestrian Plugin package provides highly customizable pedestrian actor control.
- You can change the actor position by writing a Python module
- The plugin will reload the python module when the actor pose is reset.
  - Use `Gazebo` -> `Edit` -> `Reset Model Poses (Ctrl+Shift+R)` to reset.
  - If you provide your Python module as ROS2 package, the best way to debug your module is to use `colcon build --symlink-install`

## Limitation

- `walk.dae` animation only

## Support

- ROS2 humble
- Gazebo classic 11

## Example of plugin description
```xml
<actor name="walking_actor1">
  <pose>5 5 0.0 0 0 0</pose>
  <skin>
      <filename>walk.dae</filename>
      <scale>1.0</scale>
  </skin>
  <animation name="walking">
      <filename>walk.dae</filename>
      <scale>1.0</scale>
      <interpolate_x>true</interpolate_x>
  </animation>
  <plugin name="pedestrian_plugin1" filename="libpedestrian_plugin.so">
    <module>pedestrian</module>
    <robot>robot</module>
    <velocity type="float">1.0</velocity>
    <behavior type="str">straight</behavior>
    <max_dist type="int">10</max_dist>
    <loop type="bool">True</loop>
  </plugin>
</actor>
```
### plugin parameters
- **module** : python module name to control the actor position (see next section)
- **robot** : name of the robot model
- Other parameters will be parsed as `<string, PyObject*>` map and will be passed to the specified module.
- Supported type is `int`, `float`, `str`, or `bool`.
- Do not use the keys `x`, `y`, `z`, `roll`, `pitch`, `yaw`, `dt`, and `name`

## Python module

- You can change actor position by implementing onUpdate function of the specified module.
- `pedestrian` module is provided as an example in this package.
- Make sure your specified module is in the pythone path.
- The simplest module is the following

```python
# pedestrian/pedestrian.py
import ros
def onUpdate(**args):
  args['x'] += args['dt']  # move towards +x axes at 1 m/s
  ros.info("hello")  # RCLCPP_INFO will be used to print "hello"
  return args

# pedestrian/__ini__.py
from .pedestrian import onUpdate
__all__ = ['onUpdate']
```

### `args` dict

- `x`, `y`, `z`, `roll`, `pitch`, `yaw`: actor pose
- `dt`: the time difference from the previous update
- `name`: the actor name specified in the plugin description
- `robot`: the pose of the robot if `robot` is specified as plugin params
- You need to return the updated dict or newly allocated dict including the pose keys.
- The `args` dict also includes the parameters you specified in the `<plugin>` description.
  - `velocity=1.0`, `behavior=straight`, `max_dist=10`, and `loop=True` will be included in the `args` dict with the above example plugin description.

### `ros` module API

`ros` module is provided by the plugin code (not from your ROS environment)
- `ros.info(message: str)`  : RCLCPP_INFO will be used to print the `message`

## `/people` topic

The plugin publishes pedestrian locations as `people_msgs::msg::People` message

## `/pedestrian_plugin_update` service

Once you insert this plugin, the actor can be updated via the service.
