ip_address=`ip -o route get to 8.8.8.8 | sed -n 's/.*src \([0-9.]\+\).*/\1/p'`
yarpserver --write --ip $ip_address --port 10000