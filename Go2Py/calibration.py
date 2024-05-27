import gtsam
import numpy as np
from gtsam.symbol_shorthand import X
import yaml

def Vicon2GtExtractParams(params_file):
    """
    Read the transformation matrix representing the pose of the vicon marker frame with respect to
    the robot IMU frame (B1 body frame) and return as a 4x4 numpy array.
    """
    # Initialize variables to None
    R_BtoI = None
    p_BinI = None

    # Open the file and read line by line
    with open(params_file) as f:
        lines = f.readlines()

    reading_R = False
    reading_p = False
    R_rows = []
    p_values = []

    for line in lines:
        # Check if the line starts reading R_BtoI matrix or p_BinI vector
        if "R_BtoI:" in line:
            reading_R = True
            continue
        elif "p_BinI:" in line:
            reading_p = True
            continue

        # Read the values for R_BtoI
        if reading_R:
            if line.strip():  # not an empty line
                R_rows.append(list(map(float, line.split())))
            else:
                reading_R = False
                R_BtoI = np.array(R_rows)

        # Read the values for p_BinI
        if reading_p:
            if line.strip():  # not an empty line
                p_values.append(float(line.strip()))
            else:
                reading_p = False
                p_BinI = np.array(p_values)

    # Create the 4x4 transformation matrix
    T_BtoI = np.identity(4)
    T_BtoI[0:3, 0:3] = R_BtoI
    T_BtoI[0:3, 3] = p_BinI

    return T_BtoI

def KalibrExtractExtrinsics(file_path):
    """
    Extract the extrinsic parameters from a given YAML file.
    
    Parameters:
    - file_path (str): The path to the input YAML file.
    
    Returns:
    - dict: A dictionary containing the extrinsic parameters for each camera.
            For each camera:
            - T_cx_cy: The camera extrinsic transformation from camera 'y' to camera 'x' coordinates.
                       Always with respect to the last camera in the chain.
                       (e.g. for cam1, T_c1_c0 takes cam0 to cam1 coordinates)
            - T_cam_imu: IMU extrinsics, transformation from IMU to camera coordinates (T_c_i).

    Example:
    Given a yaml file with content:
    cam0:
      ...
      T_cam_imu: [...]
    cam1:
      ...
      T_cn_cnm1: [...]
      
    The function will return a dictionary:
    {
        "cam0_T_cam_imu": [...],
        "T_c1_c0": [...]
    }
    """
    
    # Dictionary to store the extrinsic parameters
    extrinsic_params = {}

    # Open and read the YAML file
    with open(file_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

        # Iterate through the sensors in the yaml data
        for sensor, data in yaml_data.items():
            # Extract the IMU to camera transformation
            # extrinsic_params[sensor]= data['rostopic']
            if 'T_cam_imu' in data:
                extrinsic_params[f"{sensor}_T_imu"] = np.array(data['T_cam_imu'])
            
            # Extract the camera to camera transformation
            if 'T_cn_cnm1' in data:
                # Extract camera numbers from the sensor name
                current_cam_num = int(sensor.split('cam')[-1])
                last_cam_num = current_cam_num - 1  # Last camera in the chain
                
                # Change the key name to use explicit camera numbers
                current_cam_name = f"cam{current_cam_num}"
                last_cam_name = f"cam{last_cam_num}"
                key_name = f"{current_cam_name}_T_{last_cam_name}"
                extrinsic_params[key_name] = np.array(data['T_cn_cnm1'])

    return extrinsic_params

def KalibrExtractIntrinsics(file_path):
    """
    Extract the intrinsic parameters from a given YAML file.
    
    Parameters:
    - file_path (str): The path to the input YAML file.
    
    Returns:
    - dict: A dictionary containing the intrinsic parameters for each camera.
            For each camera:
            - intrinsics: Dictionary of intrinsic parameters based on the camera model.
            - distortion_coeffs: Dictionary of distortion coefficients based on the distortion model.
            - resolution: List containing camera resolution [width, height].
            - camera_model: String specifying the camera model.
            - distortion_model: String specifying the distortion model.
    """

    # Open and read the YAML file
    with open(file_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

    intrinsic_params = {}
    for sensor, data in yaml_data.items():
        sensor_data = {}
        
        # Extract camera model and intrinsics
        camera_model = data['camera_model']
        sensor_data['camera_model'] = camera_model
        intrinsics = {}
        if camera_model == 'pinhole':
            intrinsics = {
                'fu': data['intrinsics'][0],
                'fv': data['intrinsics'][1],
                'pu': data['intrinsics'][2],
                'pv': data['intrinsics'][3]
            }
        elif camera_model == 'omni':
            intrinsics = {
                'xi': data['intrinsics'][0],
                'fu': data['intrinsics'][1],
                'fv': data['intrinsics'][2],
                'pu': data['intrinsics'][3],
                'pv': data['intrinsics'][4]
            }
        # ... add similar conditions for 'ds' and 'eucm'
        
        sensor_data['intrinsics'] = intrinsics

        # Extract distortion model and coefficients
        distortion_model = data['distortion_model']
        sensor_data['distortion_model'] = distortion_model
        coeffs = {}
        if distortion_model == 'radtan':
            coeffs = {
                'k1': data['distortion_coeffs'][0],
                'k2': data['distortion_coeffs'][1],
                'r1': data['distortion_coeffs'][2],
                'r2': data['distortion_coeffs'][3]
            }
        elif distortion_model == 'equi':
            coeffs = {
                'k1': data['distortion_coeffs'][0],
                'k2': data['distortion_coeffs'][1],
                'k3': data['distortion_coeffs'][2],
                'k4': data['distortion_coeffs'][3]
            }
        # ... add similar conditions for 'fov' and 'none'

        sensor_data['distortion_coeffs'] = coeffs
        sensor_data['resolution'] = data['resolution']

        intrinsic_params[sensor] = sensor_data

    return intrinsic_params

def KalibrExtractDirectory(directory_path):
    """
    Process all YAML files in a directory and return camera details.
    
    Parameters:
    - directory_path (str): The path to the directory containing YAML files.
    
    Returns:
    - dict, dict: Two dictionaries containing extrinsic and intrinsic parameters for each camera.
    """
    
    extrinsic_all = {}
    intrinsic_all = {}

    # List all files in the directory
    for filename in os.listdir(directory_path):
        # Check if the file is a YAML file
        if filename.endswith(".yaml"):
            filepath = os.path.join(directory_path, filename)
            rig_name = os.path.splitext(filename)[0]  # Get the rig name from the filename without extension
            
            # Extract parameters using previously defined functions
            extrinsic_params = KalibrExtractExtrinsics(filepath)
            intrinsic_params = KalibrExtractIntrinsics(filepath)
            
            # Rename keys by appending rig_name
            for key in extrinsic_params.keys():
                extrinsic_all[f"{rig_name}_{key}"] = extrinsic_params[key]
                
            for key in intrinsic_params.keys():
                intrinsic_all[f"{rig_name}_{key}"] = intrinsic_params[key]

    return extrinsic_all, intrinsic_all

class ExtrinsicCalibrationManager:
    """
    A class that manages the relative transformations (SE3) between a suite of sensors using GTSAM.
    """

    def __init__(self):
        """
        Initializes an SE3ExtrinsicManager object.
        """
        self.reset()

    def reset(self):
        """
        Resets the internal state of the manager.
        """
        self.graph = gtsam.NonlinearFactorGraph()  # Graph for optimization
        self.values = gtsam.Values()  # Values in the graph
        self.name_to_id = {}  # Map of sensor name to ID
        self.id_to_name = {}  # Map of ID to sensor name
        self.latest_assigned_id = -1  # Latest ID assigned to a sensor
        self.extrinsics = None  # Optimized extrinsics

    def add(self, parent, child, R, t, confidence=1):
        """
        Add a relative transformation (SE3) between parent and child sensors ${}^{parent} T_{child}$.

        @param parent: Name of the parent sensor.
        @param child: Name of the child sensor.
        @param R: Rotation matrix (3x3) or quaternion (4x1, 1x4, 4,) [qw, qx, qy, qz].
        @param t: Translation vector [x, y, z].
        @param confidence: Confidence level for the measurement. Defaults to 1.
        """
        # Handle the case where R is a quaternion
        if R.shape == (4, 1) or R.shape == (1, 4) or R.shape == (4,):
            qw = R[0]
            qx = R[1]
            qy = R[2]
            qz = R[3]
            R = gtsam.Rot3(qw, qx, qy, qz).matrix()

        # Construct a full transformation matrix
        T = np.vstack([np.hstack([R, t.reshape(3, 1)]), np.array([0, 0, 0, 1])])

        # Handle new sensors
        if parent not in self.name_to_id.keys():
            self.latest_assigned_id += 1
            self.id_to_name[self.latest_assigned_id] = parent
            self.name_to_id[parent] = self.latest_assigned_id
            self.values.insert(X(self.name_to_id[parent]), gtsam.Pose3())

        if child not in self.name_to_id.keys():
            self.latest_assigned_id += 1
            self.id_to_name[self.latest_assigned_id] = child
            self.name_to_id[child] = self.latest_assigned_id
            self.values.insert(X(self.name_to_id[child]), gtsam.Pose3())

        # Add measurement to the graph
        Sigma = gtsam.noiseModel.Diagonal.Sigmas([confidence for _ in range(6)])
        self.graph.add(
            gtsam.BetweenFactorPose3(
                X(self.name_to_id[parent]),
                X(self.name_to_id[child]),
                gtsam.Pose3(T),
                Sigma,
            )
        )
        # Optimize the graph
        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.values, params)
        self.extrinsics = optimizer.optimize()

    def get(self, parent, child):
        """
        Get the relative transformation from the parent sensor to the child sensor.

        @param parent: Name of the parent sensor.
        @param child: Name of the child sensor.
        @return: A 4x4 matrix representing the relative transformation ${}^{reference} T_{child}$.
        """
        T_parent = self.extrinsics.atPose3(X(self.name_to_id[parent])).matrix()
        T_child = self.extrinsics.atPose3(X(self.name_to_id[child])).matrix()
        return np.linalg.inv(T_parent) @ T_child

    def get_all(self, reference):
        """
        Get transformations of all sensors relative to a given reference sensor.

        @param reference: Name of the reference sensor.
        @return: A dictionary with keys as "<sensor_name>_wrt_<reference>" and values as 4x4 transformation matrices.
        """
        assert (
            reference in self.name_to_id.keys()
        ), "Selected reference frame is not in the graph"
        return {
            f"{key}_wrt_{reference}": self.get(reference, key)
            for key in self.name_to_id.keys()
            if key != reference
        }

    def pose2qt(self, T):
        """
        Convert a 4x4 transformation matrix to quaternion and translation vector.

        @param T: A 4x4 transformation matrix.
        @return: A 4x1 quaternion.
        """
        return gtsam.Rot3(T[:3, :3]).quaternion(), T[0:3, 3]


def KalibrExtractSensorToFrameNameMap(param_file):
    '''
    Given the path to a kalibr calibration yaml file, returns a dictionary mapping sensor names in the yaml file 
    to the corresponding frame ids as published by realsense_ros ROS2 package. This has been done by inspecting the
    rostopic field in the yaml file.
    
    @ param param_file: path to a kalibr calibration yaml file
    @ return: dictionary mapping sensor names in the yaml file to the corresponding frame ids as published by realsense_ros ROS2 package.
    '''
    with open(param_file, 'r') as f:
        data = yaml.safe_load(f)

    sensor_name_to_frame_id = {}
    for sensor in data.keys():  
        tmp = data[sensor]['rostopic'].split('/')
        frame_name = tmp[2]+'_'+tmp[3]+'_optical_frame'
        sensor_name_to_frame_id[sensor] = frame_name
    return sensor_name_to_frame_id

def KalibrExtractFrameExtrinsics(camera_intrinsics_dir):
    '''
    Given the directory containing the result of kalibr calibration procedure, returns 
    a dictionary of 4x4 numpy arrays representing the extrinsics between the parent and child frames
    with parent and child frames having similar TF names as the ones published by realsense_ros ROS2 package. 

    @ param camera_intrinsics_dir: directory containing a collection yaml files from kalibr calibration procedure
    @ return: dictionary of 4x4 numpy arrays representing the extrinsics between the parent and child frames info['parent_frame_id:child_frame_id'] = pose
    '''
    intrinsics_files = os.listdir(camera_intrinsics_dir)
    info = {} # info will hold the extrinsics poses between parent frame ids and child frame ids as info['parent_frame_id:child_frame_id'] = pose
    for file in intrinsics_files:
        param_file = os.path.join(camera_intrinsics_dir,file)
        ext_params = KalibrExtractExtrinsics(param_file)
        if len(ext_params) == 0:
            continue

        map = KalibrExtractSensorToFrameNameMap(param_file)
        for pose in ext_params:
            parent_sensor_name = pose.split('_T_')[0]
            child_sensor_name = pose.split('_T_')[-1]
            if child_sensor_name == 'imu':
                child_frame_id = 'b1_imu_link'
            else:
                child_frame_id = map[child_sensor_name]

            parent_frame_id = map[parent_sensor_name]
            info[f'{parent_frame_id}:{child_frame_id}'] = ext_params[pose]
    return info