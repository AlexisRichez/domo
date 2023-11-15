docker run -d -p 9100:9116 --name snmp-exporter prom/snmp-exporter \
  --web.listen-address=":9116" \
  --web.telemetry-path="/metrics" \
  --snmp.target="<TARGET_IP>" \
  --snmp.community="<COMMUNITY_STRING>"
