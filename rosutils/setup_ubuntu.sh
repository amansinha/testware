#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)

apt install -y cowsay
export answer=y

case $answer in
  [yY]* )
    apt install -y ansible
    cd $SCRIPT_DIR/ansible
    ansible-playbook -i localhost, $SCRIPT_DIR/ansible/localhost-setup-ubuntu18.04-devpc.yml -i $SCRIPT_DIR/inventories/local-dev.ini -e AUTOWARE_DIR=$SCRIPT_DIR
    echo -e "\e[32mComplete \e[0m"
    ;;
  * )
    echo -e "\e[33mCanceled \e[0m"
    ;;
esac
