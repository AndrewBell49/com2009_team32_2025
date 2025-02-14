#! /usr/bin/env bash

ask() {
    local reply

    echo -e -n "[INPUT] $1 [y/n] >> "
    read -r reply </dev/tty
    if [[ -z $reply ]]; then
        return 1;
    elif [ "$reply" == "y" ] || [ "$reply" == "Y" ]; then
        return 0;
    else
        return 1;
    fi
}

if ! $(git status >/dev/null 2>&1); then
    echo "[ERROR] Not a git repository..."
    echo "  Make sure you are located at the root of the package directory."
    exit 255
fi

PKG_PATH=$(git rev-parse --show-toplevel)
PKG_NAME=$(basename ${PKG_PATH})

if ! ask "Initialise this ROS package with the name '${PKG_NAME}'?"; then
    echo "Exiting."
    exit 255
fi

echo "Initialising the '${PKG_NAME}' package..."

TEMPLATE_NAME=ros2_pkg_template
cd ${PKG_PATH}
mv ${TEMPLATE_NAME}_modules/ ${PKG_NAME}_modules/
mv include/${TEMPLATE_NAME}/ include/${PKG_NAME}/
sed -i '/<name>/s/'${TEMPLATE_NAME}'/'${PKG_NAME}'/' package.xml
sed -i '2 s/'${TEMPLATE_NAME}'/'${PKG_NAME}'/' CMakeLists.txt
rm -f init_pkg.sh

echo "Done."
