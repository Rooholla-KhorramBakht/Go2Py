realsense:
	@cd deploy/docker && docker build --tag go2py_realsense:latest -f Dockerfile.realsense .

hesai:
	@cd deploy && docker build --no-cache --tag go2py_hesai:latest -f docker/Dockerfile.hesai .

bridge:
	@cd deploy && docker build --no-cache --tag go2py_bridge:latest -f docker/Dockerfile.bridge .

robot_description:
	@cd deploy && docker build --no-cache --tag go2py_description:latest -f docker/Dockerfile.robot_description .

hesai_install:
	@cp deploy/services/go2py-hesai.service /etc/systemd/system/
	@systemctl enable go2py-hesai.service
	@systemctl start go2py-hesai.service

bridge_install:
	@cp deploy/services/go2py-bridge.service /etc/systemd/system/
	@systemctl enable go2py-bridge.service
	@systemctl start go2py-bridge.service

robot_description_install:
	@cp deploy/services/go2py-robot-description.service /etc/systemd/system/
	@systemctl enable go2py-robot-description.service
	@systemctl start go2py-robot-description.service
	
hesai_uninstall:
	@systemctl disable go2py-hesai.service
	@systemctl stop go2py-hesai.service
	@rm /etc/systemd/system/go2py-hesai.service

bridge_uninstall:
	@systemctl disable go2py-bridge.service
	@systemctl stop go2py-bridge.service
	@rm /etc/systemd/system/go2py-bridge.service

robot_description_uninstall:
	@systemctl disable go2py-robot-description.service
	@systemctl stop go2py-robot-description.service
	@rm /etc/systemd/system/go2py-robot-description.service