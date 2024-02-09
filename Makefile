docker:
	@docker build --no-cache --tag go2py:latest -f Dockerfile.dock .

docker_install:
	@cp deploy/scripts/go2py-hw-nodes.service /etc/systemd/system/
	@systemctl enable go2py-hw-nodes.service
	@systemctl start go2py-hw-nodes.service

docker_uninstall:
	@systemctl disable go2py-hw-nodes.service
	@systemctl stop go2py-hw-nodes.service
	@rm /etc/systemd/system/go2py-hw-nodes.service