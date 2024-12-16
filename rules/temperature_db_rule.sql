SELECT state.reported.temperature AS temperature, timestamp() AS timestamp, topic(3) AS thing_name, substring(topic(3), 10) AS version 
FROM '$aws/things/+/shadow/update/accepted' 
WHERE substring(topic(3), 0, 9) = 'incubator' AND state.reported.temperature >= 0 AND state.reported.temperature <= 100