This is part of the code for visual navigation algorithms, which fuses VO with localization method based on heterogeneous image matching.

VO uses the visual mode of ORB-SLAM3, which is written in C++. 

The image matching method uses python and data is transmitted between different codes using ZMQ.

Due to copyright issues, we have not made the code for image matching public and have left the interface 'MatchToReferenceSat()'.

This code includes:
(1) A three-thread framework: the tracking thread, the mapping thread, and the matching thread (with an interface for heterogeneous image matching).
(2) VO (the visual mode of ORB-SLAM3).
(3) Data fusion (in the run() function of the mapping thread).

The initialization work is implemented within the "FirstGeoInitialized()" and "SecondGeoInitialized()" in “LocalMapping.cc”, while the map fusion is implemented within the "GeoLocalBundleAdjustment()" in "Optimizer.cc". These functions run in the "run()" in "LocalMapping.cc" . The jacobian  is implemented in "EdgeSim3InGeoSLAM" within g2o. 

For memory reasons, we removed the Vocabulary file, which can be found in the code of ORB-SLAM3.

Compilation: ./build.sh

Run: ./Examples/Monocular/mono_video ./Vocabulary/ORBvoc.txt ./Examples/Monocular/2_960*540_20.yaml ./video.mp4
