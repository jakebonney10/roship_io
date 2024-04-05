#!/bin/bash

counter=0
while [ $counter -lt 60 ]; do
    mosquitto_pub -h localhost -t "test/topic" -m "Hello, MQTT!"
    sleep 1
    ((counter++))
done



