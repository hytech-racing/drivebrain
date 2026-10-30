for ip_addr in 192.168.203.67 192.168.10.1 192.168.3.30; do
    if ping -c 1 -W 2 $ip_addr >/dev/null 2>&1; then
        echo "Using IP address $ip_addr"
        IS_REACHABLE="true"
        SSH_OPTS="-o ControlMaster=auto -o ControlPath=/tmp/ssh-%C -o ControlPersist=60"
        TARGET=hytech@$ip_addr

        ssh $SSH_OPTS $TARGET true   # authenticates once, opens master connection
        ssh $SSH_OPTS $TARGET "sudo systemctl stop drivebrain-service.service"
        echo "Drivebrain has been stopped."
        scp $SSH_OPTS build-arm/drivebrain          $TARGET:/home/hytech/drivebrain
        scp $SSH_OPTS config/drivebrain_config.json $TARGET:/home/hytech/drivebrain_config.json
        scp $SSH_OPTS dbc/ht_can.dbc                $TARGET:/home/hytech/hytech.dbc
        echo "Drivebrain code has successfully been copied."
        ssh $SSH_OPTS $TARGET "sudo systemctl start drivebrain-service.service"
        echo "Drivebrain restart successful."
        break
    fi
done

if [[ "$IS_REACHABLE" != "true" ]]; then
    echo "Drivebrain is not reachable"
fi
