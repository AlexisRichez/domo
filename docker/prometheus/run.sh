docker build -t my-prometheus .

docker run -d \
  -v /nfs_share/prometheus_data:/prometheus \
  -p 9090:9090 \
  my-prometheus
