#/bin/bash
#QT_VER="$(ls ~/Qt/ | grep 5 -m1)"
#printf "${HOME}/Qt/${QT_VER}/gcc_64/"

QT_VER="$(ls /opt/Qt*/ | grep 5 -m1)"
printf "/opt/Qt5.12.12/${QT_VER}/gcc_64/"

