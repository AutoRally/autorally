#!/bin/bash

host=$MASTER_HOSTNAME
if [ $(hostname) == "$MASTER_HOSTNAME" ]
then
  host="localhost"
fi

chronyc -h $host <<EOF > chronyClients
password muri123
clients
EOF
