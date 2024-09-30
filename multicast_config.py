import argparse
import subprocess


def configure_multicast(device_name):
    """
    Configures multicast settings for a given network device.

    @param device_name: The name of the network device to configure.
    @type device_name: str

    @raises subprocess.CalledProcessError: If the execution of any command fails.

    @note:
        This function requires administrative privileges to execute commands like 'route' and 'ifconfig'.
        Run the Python script as a superuser or provide the necessary credentials.

    @example:
        device = 'eth0'
        configure_multicast(device)
    """

    add_route_command = [
        "route",
        "add",
        "-net",
        "224.0.0.0",
        "netmask",
        "240.0.0.0",
        "dev",
        device_name,
    ]
    enable_multicast_command = ["sudo", "ifconfig", device_name, "multicast"]

    subprocess.run(add_route_command, check=True)
    subprocess.run(enable_multicast_command, check=True)


def main():
    """
    Main entry point of the script.
    Parses the command-line arguments and invokes the configuration function.
    """

    # Create an argument parser
    parser = argparse.ArgumentParser(
        description="Configure multicast settings for a network interface."
    )

    # Add the interface name argument
    parser.add_argument(
        "interface", type=str, help="The name of the network interface to configure."
    )

    # Parse the command-line arguments
    args = parser.parse_args()

    # Invoke the configuration function
    configure_multicast(args.interface)


if __name__ == "__main__":
    main()
