#!/bin/bash
python -m pip install inflect
mkdir CR2020_GRUPPO26_WS
cd CR2020_GRUPPO26_WS
mkdir src
catkin init
catkin build
cd src
wget -O gdrivedl 'https://f.mjh.nz/gdrivedl'
chmod +x gdrivedl
export fileid=1EOrH6W5axf_4VyewUbGhsCZ4pqJvBKNS
export filename='CR2020_GRUPPO26.7z'
./gdrivedl $fileid $filename
rm gdrivedl
7z X $filename
rm $filename
cd ..
echo 'D=$(realpath src/pynaoqi-python2.7-2.5.7.1-linux64)' >> devel/setup.bash
echo 'export PYTHONPATH=${PYTHONPATH}:$D/lib/python2.7/site-packages' >> devel/setup.bash
echo 'export DYLD_LIBRARY_PATH=${DYLD_LIBRARY_PATH}:$D/lib' >> devel/setup.bash
catkin config --blacklist pepper_pkg pepper_msgs
catkin build
catkin config --no-blacklist
catkin build pepper_pkg pepper_msgs
cd src/pepper_pkg/src
cd camera_node
chmod +x camera_node
cd ..
cd detector_node
chmod +x detector_node
cd ..
cd head_node
chmod +x head_node
cd ..
cd speaker_node
chmod +x speaker_node
cd ../../../..
source devel/setup.bash
