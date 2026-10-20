#!/usr/bin/env sh
set -e

HOST="${DEPLOY_HOST:-hytech@192.168.10.1}"
export SSHPASS="${SSHPASS:-hytech}"

# reuse a single ssh connection for all copies, auto-accept new host keys
CTL="/tmp/drivebrain-deploy-%r@%h:%p"
SSH_OPTS="-o StrictHostKeyChecking=accept-new -o ControlMaster=auto -o ControlPath=$CTL -o ControlPersist=30"

if command -v sshpass >/dev/null 2>&1; then
  SCP="sshpass -e scp $SSH_OPTS"
else
  echo "sshpass not found (brew install sshpass); you will be prompted for the password once" >&2
  SCP="scp $SSH_OPTS"
fi

$SCP build-arm/drivebrain "$HOST":/home/hytech/drivebrain
$SCP config/drivebrain_config.json "$HOST":/home/hytech/drivebrain_config.json
$SCP build-arm/_deps/ht_can-src/dbc/ht_can.dbc "$HOST":/home/hytech/hytech.dbc

ssh $SSH_OPTS -O exit "$HOST" 2>/dev/null || true
echo "deployed to $HOST"
