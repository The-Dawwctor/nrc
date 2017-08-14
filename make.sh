set -e

# Update sai2-common
cd sai2-common
git pull origin master
mkdir -p build_rel
cd build_rel
cmake ..
make -j4
cd ../..

# Build cs225a
mkdir -p build
cd build
cmake ..
make -j4
cd ..

# Download resource files
mkdir -p resources
cd resources
if [ ! -d "puma_graphics" ]; then
	curl -L http://cs.stanford.edu/groups/manips/teaching/cs225a/resources/puma_graphics.zip -o puma_graphics.zip
	unzip puma_graphics.zip
	rm puma_graphics.zip
fi
if [ ! -d "kuka_iiwa_graphics" ]; then
	curl -L http://cs.stanford.edu/groups/manips/teaching/cs225a/resources/kuka_iiwa_graphics.zip -o kuka_iiwa_graphics.zip
	unzip kuka_iiwa_graphics.zip
	rm kuka_iiwa_graphics.zip
fi
cd ..

cd bin
if [ -f "kuka_iiwa_driver" ]; then
	cd resources/kuka_iiwa_driver
	if [ ! -e "kuka_iiwa_graphics" ]; then
		ln -s ../../../resources/kuka_iiwa_graphics .
	fi
	cd ../..
fi
if [ -f "kuka_hold_pos" ]; then
	cd resources/kuka_hold_pos
	if [ ! -e "kuka_iiwa_graphics" ]; then
		ln -s ../../../resources/kuka_iiwa_graphics .
	fi
	cd ../..
fi
cd ..

# Insert helper scripts into bin directory
cd bin

# Make script
cat <<EOF > make.sh
cd ..
mkdir -p build
cd build
cmake ..
make -j4
cd ../bin
EOF
chmod +x make.sh

# Run generic controller script
if [ -f "nrc" ]; then
    cat <<EOF > run_nrc.sh
if [ "\$#" -lt 3 ]; then
    cat <<EOM
This script calls ./visualizer, ./simulator, and ./nrc simultaneously.
All the arguments after the controller will be passed directly to it.

Usage: sh run_nrc.sh <path-to-world.urdf> <path-to-robot.urdf> <robot-name>
EOM
else
    trap 'kill %1; kill %2' SIGINT
    trap 'kill %1; kill %2' EXIT
    ./simulator \$1 \$2 \$3 > simulator.log & ./visualizer \$1 \$2 \$3 > visualizer.log & ./nrc \$1 \$2 \$3
fi
EOF
    chmod +x run_nrc.sh
fi

cd ..
