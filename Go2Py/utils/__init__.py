import os
import tempfile
from Go2Py import ASSETS_PATH
def set_cyclonedds_config(interface_name):
    # Generate the XML configuration
    xml_content = f"""<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="{interface_name}" priority="default" multicast="default" />
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>"""

    # Create a temporary XML file
    cfg_file_path = os.path.join(ASSETS_PATH, 'cyclonedds.xml')
    with open(cfg_file_path, 'w') as f:
        f.write(xml_content)
    
    # Set the environment variable
    os.environ['CYCLONEDDS_URI'] = f'file://{cfg_file_path}'