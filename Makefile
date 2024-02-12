docker_dock:
	@docker build --no-cache --tag go2py_dock_hw:latest -f Dockerfile.dock .

docker_robot:
	@docker build --no-cache --tag go2py_robot_hw:latest -f Dockerfile.robot .

docker_dock_install:
	@cp deploy/scripts/go2py-dock-hw-nodes.service /etc/systemd/system/
	@systemctl enable go2py-dock-hw-nodes.service
	@systemctl start go2py-dock-hw-nodes.service

docker_dock_uninstall:
	@systemctl disable go2py-dock-hw-nodes.service
	@systemctl stop go2py-dock-hw-nodes.service
	@rm /etc/systemd/system/go2py-dock-hw-nodes.service

docker_robot_install:
	@cp deploy/scripts/go2py-robot-hw-nodes.service /etc/systemd/system/
	@systemctl enable go2py-robot-hw-nodes.service
	@systemctl start go2py-robot-hw-nodes.service

docker_robot_uninstall:
	@systemctl disable go2py-robot-hw-nodes.service
	@systemctl stop go2py-robot-hw-nodes.service
	@rm /etc/systemd/system/go2py-robot-hw-nodes.service