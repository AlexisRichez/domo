docker run -d -p 9116:9116 --name snmp-exporter -v ./snmp.yml:/etc/snmp_exporter/snmp.yml prom/snmp-exporter
