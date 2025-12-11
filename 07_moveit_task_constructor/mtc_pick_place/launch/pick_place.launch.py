from json import dumps

import rclpy
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.node import Node as RclpyNode
from rclpy.logging import get_logger
from rcl_interfaces.srv import ListParameters, GetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType


NODE_NAME = "config_restorer"
logger = get_logger(NODE_NAME)


def unflatten_dict(flat_dict: dict):
    """
    Recursively reconstructs a nested dictionary from a flat dictionary
    with dot-separated keys.

    For example, {"a.b.c": 1} becomes {"a": {"b": {"c": 1}}}.

    Args:
        flat_dict (dict): A dictionary with dot-separated string keys.

    Returns:
        dict: The reconstructed nested dictionary.
    """
    nested_dict = {}
    for key, value in flat_dict.items():
        parts = key.split(".")
        d = nested_dict
        # Iterate through parts to create nested dictionaries
        for part in parts[:-1]:
            if part not in d:
                d[part] = {}
            d = d[part]
        # Set the value at the final key
        d[parts[-1]] = value
    return nested_dict


class MetaConfigRestorer(RclpyNode):
    """
    A temporary ROS 2 node that dynamically discovers, fetches, and correctly
    types all parameters from a target node (e.g., /move_group) to
    reconstruct its full configuration.
    """

    def __init__(self, target_node_name="/move_group", node_name=NODE_NAME):
        """
        Initializes the node and creates clients for ROS 2 parameter services.

        Args:
            target_node_name (str): The fully-qualified name of the node to get parameters from.
            node_name (str): The name for this temporary node.
        """
        super().__init__(node_name)

        # Create service clients with fully-qualified service names
        self.list_client = self.create_client(
            ListParameters, f"{target_node_name}/list_parameters"
        )
        self.get_client = self.create_client(
            GetParameters, f"{target_node_name}/get_parameters"
        )

    def get_python_value(self, param_value: ParameterValue):
        """
        Safely converts an rcl_interfaces.msg.ParameterValue message into a
        standard Python data type.

        Args:
            param_value (ParameterValue): The ROS 2 parameter value message.

        Returns:
            The corresponding Python value (e.g., str, int, bool, list),
            or None if the type is not handled.
        """
        match param_value.type:
            # --- Single Value Types ---
            case ParameterType.PARAMETER_STRING:
                return param_value.string_value
            case ParameterType.PARAMETER_BOOL:
                return param_value.bool_value
            case ParameterType.PARAMETER_INTEGER:
                return param_value.integer_value
            case ParameterType.PARAMETER_DOUBLE:
                return param_value.double_value
            # --- Array Value Types ---
            # Note: We cast the ROS 2 array types to standard Python lists
            case ParameterType.PARAMETER_STRING_ARRAY:
                return list(param_value.string_array_value)
            case ParameterType.PARAMETER_BOOL_ARRAY:
                return list(param_value.bool_array_value)
            case ParameterType.PARAMETER_INTEGER_ARRAY:
                return list(param_value.integer_array_value)
            case ParameterType.PARAMETER_DOUBLE_ARRAY:
                return list(param_value.double_array_value)
            # --- Default Case for Unhandled Types ---
            # The underscore '_' is a wildcard that matches any case not
            # already handled, serving as the default case.
            case _:
                logger.warn(
                    f"Unhandled or unknown parameter type encountered: {param_value.type}"
                )
                return None

    def restore_and_reconstruct_config(self):
        """
        Executes the full process of discovering, fetching, and reconstructing
        the nested parameter dictionary from the target node.

        Returns:
            dict: The fully reconstructed, nested parameter dictionary, or None on failure.
        """
        # 1. Wait for the parameter services on the target node to be available.
        if not self.list_client.wait_for_service(timeout_sec=5.0):
            logger.error(
                f"'{self.list_client.srv_name}' service not available. Is move_group running?"
            )
            return None
        if not self.get_client.wait_for_service(timeout_sec=5.0):
            logger.error(
                f"'{self.get_client.srv_name}' service not available. Is move_group running?"
            )
            return None

        # 2. Asynchronously call the 'list_parameters' service to get all parameter names.
        logger.info(f"Listing all parameters from '{self.get_client.srv_name}'...")
        list_req = ListParameters.Request()
        future = self.list_client.call_async(list_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            logger.error(f"Failed to call '{self.list_client.srv_name}'.")
            return None

        all_names = future.result().result.names
        logger.info(f"Discovered {len(all_names)} parameters.")

        # 3. Asynchronously call 'get_parameters' in a single batch to get all values.
        logger.info("Fetching all parameter values in a single batch call...")
        get_req = GetParameters.Request()
        get_req.names = all_names
        future = self.get_client.call_async(get_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            logger.error(f"Failed to call '{self.get_client.srv_name}'.")
            return None

        all_values = future.result().values
        logger.info(f"Parameter values grabbed:\n{all_values}")

        # 4. Convert the flat list of ROS messages into a flat dictionary of Python types.
        flat_python_dict = {}
        for name, value_msg in zip(all_names, all_values):
            python_val = self.get_python_value(value_msg)
            if python_val is not None:
                flat_python_dict[name] = python_val

        # 5. Reconstruct the final nested dictionary from the flat dictionary.
        logger.info(
            "Reconstructing nested dictionary from flattened parameter names..."
        )
        nested_config = unflatten_dict(flat_python_dict)
        logger.info(
            f"Reconstructed parameter's dictionary:\n{dumps(nested_config, indent=2)}"
        )

        # 6. Final sanity check for the most critical parameter.
        if "robot_description" not in nested_config:
            logger.error(
                "CRITICAL: 'robot_description' was not found in the restored config. Aborting launch."
            )
            return None

        logger.info(
            "Successfully restored and reconstructed the full MoveIt configuration."
        )
        return nested_config


def generate_launch_description():
    """
    The main entry point for the ROS 2 launch system.

    This function initializes a temporary node to fetch the configuration
    from a running move_group, then uses that config to launch the target
    C++ node.
    """
    # Initialize rclpy to allow for node creation and service calls.
    rclpy.init()

    # Create an instance of our temporary node and execute the restoration.
    restorer = MetaConfigRestorer()
    reconstructed_config = restorer.restore_and_reconstruct_config()

    # Cleanly destroy the temporary node and shut down the rclpy context.
    restorer.destroy_node()
    rclpy.shutdown()

    # If the restoration process failed, return an empty launch description.
    if not reconstructed_config:
        logger.error("Configuration restoration failed. No nodes will be launched.")
        return LaunchDescription([])

    # Define the target C++ node to launch, passing the reconstructed config
    # as its parameters.
    mtc_node = Node(
        package="mtc_pick_place",
        executable="mtc_node",
        output="screen",
        parameters=[reconstructed_config],
    )

    # Return the final LaunchDescription containing the configured node.
    return LaunchDescription([mtc_node])
