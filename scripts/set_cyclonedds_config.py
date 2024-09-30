from Go2Py.utils import set_cyclonedds_config
import argparse

def main():
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="Set CycloneDDS configuration.")
    parser.add_argument("network_interface", help="The name of the network interface used to communicate with the robot.")
    # Parse the arguments
    args = parser.parse_args()
    # Set the CycloneDDS configuration
    set_cyclonedds_config(args.network_interface)

if __name__ == "__main__":
    main()