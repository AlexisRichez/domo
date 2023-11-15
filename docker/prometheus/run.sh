docker build -t my-prometheus .

docker run -d \
  -v /mnt/infra/prometheus:/prometheus \
  -p 9090:9090 \
  my-prometheus
