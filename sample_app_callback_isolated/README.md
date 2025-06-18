# sample_app_callback_isolated
A sample application to demonstrate the Executors and ComponentContainers in the `callback_isolated_executor` package.

## Launch without ComponentContainer (from main function)
```bash
$ ros2 run sample_app_callback_isolated sample_node_main
```

## Launch with ComponentContainer
Launch the component container and load the node at the same time.
```bash
$ ros2 launch sample_app_callback_isolated sample_node.launch.xml
```

Or, load the node to the exsiting component container.
```bash
$ ros2 run callback_isolated_executor component_container_callback_isolated --ros-args --remap __node:=sample_container
$ ros2 launch sample_app_callback_isolated load_sample_node.launch.xml
```
