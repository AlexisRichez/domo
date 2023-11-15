mkdir -p home/docker/node-exporter
cd home/docker/node-exporter

# Run the node-exporter docker container
docker run -d \
--name="node-exporter" \
--net="host" \
--pid="host" \
-v "/:/host:ro,rslave" \
--restart=always \
rycus86/prometheus-node-exporter:armv6 --path.rootfs=/host
# Node exporter is installed on 9100 port by default
